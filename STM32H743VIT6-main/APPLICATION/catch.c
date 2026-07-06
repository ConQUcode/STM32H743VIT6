/**
 * @file catch.c
 * @brief 抓取机构控制: 飞特舵机夹爪、达妙 4340 旋转电机、DJI 3508 升降电机和 PC2 气缸。
 *
 * 主要流程:
 *   1. CatchInit() 初始化飞特舵机、达妙电机、3508 电机，并设置 PC2 初始状态。
 *   2. CatchTask() 周期执行 LiftInit()，升降机构归零完成后才响应遥控器命令。
 *   3. 左拨杆=1、右拨杆=1 时执行飞特夹爪抓取，并带飞特电流保护。
 *   4. 左拨杆=1、右拨杆=2 时达妙转到 level_pos，随后执行 3508 堵转复位流程。
 *   5. 左拨杆=1、右拨杆=3 时执行释放流程: 3508 动作、PC2 气缸动作、飞特大张开。
 */

#include "catch.h"
#include "daemon.h"
#include "remote.h"
#include "feite_motor.h"
#include "DJI_motor.h"
#include "cmsis_os.h"
#include "dm_motor.h"
#include <math.h>

/* 飞特舵机实例: 当前只使用 FT_3 控制夹爪，其余实例暂时预留。 */
static FeiteMotor_Instance *FT_1, *FT_2, *FT_3, *FT_4;

/* DJI 3508 用于升降机构；达妙 4340 使用 CAN1，tx_id=1，rx_id=0x11。 */
static DJIMotor_Instance *DJM2006, *DJM3508;
static DMMotor_Instance *dm4310_motor;

/* 达妙初始位置，单位 rad。 */
static float init_pos = -1.8f;
/* 达妙 level 位置: 从 init_pos 正方向旋转 90 度后的目标位置，单位 rad。 */
static float level_pos = -0.2292f;
/* 达妙位置速度模式下的目标速度。 */
static float dm4310_target_vel = 1.5f;

/* KEY0 触发动作后 PC2 延时切换的时间，单位 ms。 */
#define CATCH_PC2_DELAY_MS 2000U
/* 判断达妙已经到达 level_pos 的允许误差，单位 rad。 */
#define DM_LEVEL_POS_TOL_RAD      0.15f
/* 3508 默认角度目标；不在状态机流程内时保持在该位置。 */
#define LIFT_DEFAULT_ANGLE_REF    6000
/* 左1右2流程中，3508 堵转复位阶段的速度环目标。 */
#define LIFT_SECOND_SPEED_REF     (-3000)
/* 左1右2流程开始时，3508 先上升到该角度后再执行达妙 level 流程。 */
#define LIFT_SECOND_PRE_UP_REF    15000.0f
/* 左1右2流程开始时，判断 3508 已经上升到位的角度死区。 */
#define LIFT_SECOND_PRE_UP_TOL    400.0f
/* 左1右2流程中，3508 判定堵转/接触限位的电流阈值。 */
#define LIFT_SECOND_CURRENT_LIMIT 1500
/* 左1右2流程中，3508 进入速度环后延时多久再开始检测电流，单位 ms。 */
#define LIFT_SECOND_CURRENT_DELAY_MS 500U
/* 左1右2流程完成后，KEY0/KEY1 手动控制 3508 上下运动的速度目标。 */
#define LIFT_MANUAL_SPEED_REF     5000
/* 左1右3释放流程中，3508 先在当前角度基础上上升的增量。 */
#define LIFT_RELEASE_RAISE_DELTA  2500.0f
/* 左1右3释放流程中，PC2 动作前 3508 下降到的第一目标角度。 */
#define LIFT_RELEASE_DOWN_REF     7000.0f
/* 左1右3释放流程中，PC2 拉低后 3508 最终下降保持的角度。 */
#define LIFT_RELEASE_FINAL_DOWN_REF 6000.0f
/* 左1右3释放流程中，3508 到位判断的角度死区。 */
#define LIFT_RELEASE_POS_TOL      500.0f
/* 3508 下降到该角度以下时触发 PC2 拉低。 */
#define LIFT_RELEASE_DOWN_TRIGGER (LIFT_RELEASE_DOWN_REF + LIFT_RELEASE_POS_TOL)
/* 左1右3释放流程中，3508 上升到位后等待 PC2 后续动作的时间，单位 ms。 */
#define LIFT_RELEASE_PC2_DELAY_MS 1000U
/* 左1右3释放流程中，PC2 拉低后等待多久再执行 FeiteBigOpen()，单位 ms。 */
#define LIFT_RELEASE_BIGOPEN_DELAY_MS 800U
/* 飞特夹爪抓取时的力矩限制值。 */
#define FEITE_CATCH_TORQUE        1000U
/* 飞特夹爪抓住后保持阶段的力矩限制值。 */
#define FEITE_HOLD_TORQUE         800U
/* 飞特夹爪张开时的力矩限制值。 */
#define FEITE_OPEN_TORQUE         800U
/* 飞特夹爪抓取时允许高力矩持续的最长时间，单位 ms。 */
#define FEITE_TORQUE_PROTECT_MS   1200U
/* 飞特夹爪抓取保护中读取电流的周期，单位 ms。 */
#define FEITE_CURRENT_CHECK_MS    50U
/* 周期读取飞特位置、速度、电流反馈的间隔，单位 ms。 */
#define FEITE_FEEDBACK_READ_MS    100U
/* 飞特电流保护阈值，对应 FT_3->measure.current_signed 的原始值。 */
#define FEITE_CURRENT_LIMIT_RAW   350U
/* 连续超过电流阈值多少次后进入保护。 */
#define FEITE_CURRENT_LIMIT_COUNT 3U

/* 升降机构初始化标志: is_init_3508 为 1 后 CatchTask 才执行遥控器状态机。 */
static int8_t is_init_2006 = 0;
static int8_t is_init_3508 = 0;

/* 调试/传感器变量，保留给外部观察使用。 */
int IR_sensor_level;
float control;

/* 左1右2流程完成后，记录 KEY0/KEY1 手动控制 3508 的方向和按键边沿。 */
static int8_t lift_manual_dir = 0;
static uint8_t lift_key0_last = 1U;
static uint8_t lift_key1_last = 1U;

/* 初始化飞特舵机总线和 ID=3 的夹爪舵机。 */
static void FeiteMotorsInit(void)
{
    FeiteMotor_Bus_Init_Config_s bus_config = {
        .huart = &huart1,
        .tx_timeout_ms = 2U,
        .rx_timeout_ms = 2U,
        .endian = FEITE_ENDIAN_LITTLE,
        .reply_level = 0U,
    };
    FeiteMotor_Bus_s *bus = FeiteMotorBusInit(&bus_config);

    FeiteMotor_Init_Config_s config = {
        .bus = bus,
        .model = FEITE_MODEL_HLS_SCS,
        .init_position = 2200,
        .init_speed = 500,
        .init_acc = 20,
        .init_torque = 1500,
        .raw_to_deg = FEITE_DEFAULT_RAW_TO_DEG,
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    };


    config.id = 3;
    FT_3 = FeiteMotorInit(&config);
}

static void DJIMotorsInit(void)
{
 Motor_Init_Config_s dm_config = {
        .can_init_config = {
            .fdcan_handle = &hfdcan1,
            .tx_id = 1,
            .rx_id = 0x11,
        },
        .controller_setting_init_config = {
            .motor_reverse_flag     = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag  = FEEDBACK_DIRECTION_NORMAL,
            .angle_mode             = MOTOR_ANGLE_MODE_SINGLE_TURN,
            .close_loop_type        = OPEN_LOOP,
        },
        .motor_type = DM4340,
    };

    dm4310_motor = DMMotorInit(&dm_config);

    if (dm4310_motor != NULL) {
        DMMotorSetControlMode(dm4310_motor, DM_MODE_POS_VEL );
        DMMotorSetPosVelRef(dm4310_motor, init_pos, dm4310_target_vel);
        DMMotorEnable(dm4310_motor);
    }

    Motor_Init_Config_s M3508_config = {
        .can_init_config = {
            .fdcan_handle = &hfdcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, .Ki = 0.18, .Kd = 0.45,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement | PID_DerivativeFilter |
                           PID_ErrorHandle,
                .IntegralLimit = 7000, .MaxOut = 22000, .Derivative_LPF_RC = 0.01,
            },
            .speed_PID = {
                .Kp = 5, .Ki = 0.05, .Kd = 0.02,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 9000, .MaxOut = 22000,
            },
            .current_PID = {
                .Kp = 0.5, .Ki = 0.01, .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000, .MaxOut = 50000,
            },
        },
        .controller_setting_init_config = {
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
        .motor_type = M3508,
    };
    DJM3508 = DJIMotorInit(&M3508_config);
    DJIMotorStop(DJM3508);
}

static void LiftInit(void)
{
    if (!is_init_3508) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
        DJIMotorEnable(DJM3508);
        DJIMotorOuterLoop(DJM3508, SPEED_LOOP);
        DJIMotorSetRef(DJM3508, -3000);

        if (abs(DJM3508->measure.real_current) > 3000) {
            osDelay(5);
            if (abs(DJM3508->measure.real_current) > 3000) {
                DJIMotorStop(DJM3508);
                DJIMotorReset(DJM3508);
                DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
							  DJIMotorEnable(DJM3508);
                DJIMotorSetRef(DJM3508, 0);
							   FeiteOpen();
							  DJIMotorSetRef(DJM3508, LIFT_DEFAULT_ANGLE_REF);
                is_init_3508 = 1;
            }
        }
    }
}


static void FeiteCatch(void)
{

    FeiteMotorSetRef(FT_3, 1930);
    FeiteMotorSetSpeed(FT_3, 1000);
    FeiteMotorSetAcc(FT_3, 1000);

}


 void FeiteOpen(void)
{

    FeiteMotorSetRef(FT_3, 2500);
    FeiteMotorSetSpeed(FT_3, 500);
    FeiteMotorSetAcc(FT_3, 20);
    FeiteMotorSetTorque(FT_3, FEITE_OPEN_TORQUE);


}

 void FeiteBigOpen(void)
{

    FeiteMotorSetRef(FT_3, 3100);
    FeiteMotorSetSpeed(FT_3, 1000);
    FeiteMotorSetAcc(FT_3, 1000);
    FeiteMotorSetTorque(FT_3, FEITE_OPEN_TORQUE);


}

static void FeiteReadFeedbackPeriodically(void)
{
    static uint32_t last_read_time = 0U;

    if (FT_3 == NULL) {
        return;
    }

    if ((uint32_t)(HAL_GetTick() - last_read_time) < FEITE_FEEDBACK_READ_MS) {
        return;
    }

    last_read_time = HAL_GetTick();
    (void)FeiteMotorReadFeedback(FT_3);
    (void)FeiteMotorReadCurrent(FT_3);
}


void CatchInit(void)
{

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    FeiteMotorsInit();
    DJIMotorsInit();
  	
}

void CatchTask(void)
{
    static uint32_t catch_start_time = 0;
    static uint32_t feite_catch_start_time = 0;
    static uint32_t feite_current_check_time = 0;
    static uint8_t is_timing = 0;
    static uint8_t level_action_started = 0U;
    static uint32_t lift_second_start_time = 0U;
    static float lift_hold_angle = 0.0f;
    static uint8_t release_action_started = 0U;
    static float release_raise_target = 0.0f;
    static uint32_t release_wait_start_time = 0U;
    static uint8_t feite_catch_started = 0U;
    static uint8_t feite_over_current_count = 0U;
    static uint8_t feite_protected = 0U;

    LiftInit();
    FeiteReadFeedbackPeriodically();

    if (!is_init_3508) {
        FeiteMotorControl();
        return;
    }

    if (remote_data != NULL) {
        if ((remote_data->switch_left == 1U) &&
            ((remote_data->switch_right == 1U) || (remote_data->switch_right == 2U) || (remote_data->switch_right == 3U))) {
            if (remote_data->switch_right == 1U) {
                FeiteCatch();

                /*
                 * 飞特抓取保护:
                 * 1. 第一次进入抓取时设置 FEITE_CATCH_TORQUE，并记录起始时间。
                 * 2. 每隔 FEITE_CURRENT_CHECK_MS 读取一次飞特电流。
                 * 3. 电流连续 FEITE_CURRENT_LIMIT_COUNT 次超过 FEITE_CURRENT_LIMIT_RAW 后进入保护。
                 * 4. 进入保护或高力矩持续超过 FEITE_TORQUE_PROTECT_MS 后，切到 FEITE_HOLD_TORQUE 保持。
                 */
            if (feite_catch_started == 0U) {
                feite_catch_started = 1U;
                feite_catch_start_time = HAL_GetTick();
                feite_current_check_time = HAL_GetTick();
                feite_over_current_count = 0U;
                feite_protected = 0U;
                FeiteMotorSetTorque(FT_3, FEITE_CATCH_TORQUE);
            } else if ((uint32_t)(HAL_GetTick() - feite_current_check_time) >= FEITE_CURRENT_CHECK_MS) {
                feite_current_check_time = HAL_GetTick();
                if (FeiteMotorReadCurrent(FT_3) == HAL_OK) {
                    int16_t current = FT_3->measure.current_signed;

                    if (current < 0) {
                        current = (int16_t)(-current);
                    }

                    if ((uint16_t)current >= FEITE_CURRENT_LIMIT_RAW) {
                        if (feite_over_current_count < FEITE_CURRENT_LIMIT_COUNT) {
                            feite_over_current_count++;
                        }
                    } else {
                        feite_over_current_count = 0U;
                    }

                    if (feite_over_current_count >= FEITE_CURRENT_LIMIT_COUNT) {
                        feite_protected = 1U;
                    }
                }
            }

            if ((feite_protected != 0U) ||
                ((uint32_t)(HAL_GetTick() - feite_catch_start_time) >= FEITE_TORQUE_PROTECT_MS)) {
                FeiteMotorSetTorque(FT_3, FEITE_HOLD_TORQUE);
            }
            } else {
                feite_catch_started = 0U;
                feite_over_current_count = 0U;
                feite_protected = 0U;
            }

            /*
             * 左1右2流程:
             * 1. 达妙先转到 level_pos，转到位前 PC2 保持高电平。
             * 2. 达妙到位后 PC2 拉低，3508 进入速度环向负方向运动。
             * 3. 延时 LIFT_SECOND_CURRENT_DELAY_MS 后开始检测 3508 电流，超过阈值后保持当前角度。
             * 4. 流程完成后，KEY0/KEY1 可临时切换 3508 到速度环手动上/下，松开后保持当前角度。
             */
            if ((remote_data->switch_left == 1U) &&
                (remote_data->switch_right == 2U)) {
                release_action_started = 0U;
                release_wait_start_time = 0U;
                if (level_action_started == 0U) {
                    level_action_started = 4U;
                    catch_start_time = HAL_GetTick();
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
                }

                if (level_action_started == 4U) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
                    DJIMotorEnable(DJM3508);
                    DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                    DJIMotorSetRef(DJM3508, LIFT_SECOND_PRE_UP_REF);

                    if (DJM3508->measure.total_angle >= (LIFT_SECOND_PRE_UP_REF - LIFT_SECOND_PRE_UP_TOL)) {
                        level_action_started = 1U;
                    }
                }

                if (level_action_started == 1U) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
                    DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                    DJIMotorSetRef(DJM3508, LIFT_DEFAULT_ANGLE_REF);
                }

                if ((level_action_started != 4U) && (dm4310_motor != NULL)) {
                    DMMotorSetPosVelRef(dm4310_motor, level_pos, dm4310_target_vel);
                    if (dm4310_motor->measure.state != 1U) {
                        DMMotorEnable(dm4310_motor);
                    }

                    if ((level_action_started == 1U) &&
                        (fabsf(dm4310_motor->measure.position_rad - level_pos) <= DM_LEVEL_POS_TOL_RAD)) {
                        level_action_started = 2U;
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
                        DJIMotorEnable(DJM3508);
                        DJIMotorOuterLoop(DJM3508, SPEED_LOOP);
                        DJIMotorSetRef(DJM3508, LIFT_SECOND_SPEED_REF);
                        lift_second_start_time = HAL_GetTick();
                    }
                }

                if (level_action_started == 2U) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
                    DJIMotorOuterLoop(DJM3508, SPEED_LOOP);
                    DJIMotorSetRef(DJM3508, LIFT_SECOND_SPEED_REF);

                    if (((uint32_t)(HAL_GetTick() - lift_second_start_time) >= LIFT_SECOND_CURRENT_DELAY_MS) &&
                        (abs(DJM3508->measure.real_current) > LIFT_SECOND_CURRENT_LIMIT)) {
                        level_action_started = 3U;
                        lift_hold_angle = DJM3508->measure.total_angle;
                        DJIMotorStop(DJM3508);
                        DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                        DJIMotorEnable(DJM3508);
                        DJIMotorSetRef(DJM3508, lift_hold_angle);
                    }
                } else if (level_action_started == 3U) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
                    DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                    DJIMotorSetRef(DJM3508, lift_hold_angle);

                    if ((lift_key0_last == 1U) && (remote_data->KEY[0] == 0U)) {
                        lift_manual_dir = 1;
                        DJIMotorOuterLoop(DJM3508, SPEED_LOOP);
                        DJIMotorSetRef(DJM3508, LIFT_MANUAL_SPEED_REF);
                    } else if ((lift_key0_last == 0U) && (remote_data->KEY[0] == 1U) && (lift_manual_dir == 1)) {
                        lift_manual_dir = 0;
                        lift_hold_angle = DJM3508->measure.total_angle;
                        DJIMotorStop(DJM3508);
                        DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                        DJIMotorEnable(DJM3508);
                        DJIMotorSetRef(DJM3508, lift_hold_angle);
                    }

                    if ((lift_key1_last == 1U) && (remote_data->KEY[1] == 0U)) {
                        lift_manual_dir = -1;
                        DJIMotorOuterLoop(DJM3508, SPEED_LOOP);
                        DJIMotorSetRef(DJM3508, -LIFT_MANUAL_SPEED_REF);
                    } else if ((lift_key1_last == 0U) && (remote_data->KEY[1] == 1U) && (lift_manual_dir == -1)) {
                        lift_manual_dir = 0;
                        lift_hold_angle = DJM3508->measure.total_angle;
                        DJIMotorStop(DJM3508);
                        DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                        DJIMotorEnable(DJM3508);
                        DJIMotorSetRef(DJM3508, lift_hold_angle);
                    }

                    if (lift_manual_dir > 0) {
                        DJIMotorOuterLoop(DJM3508, SPEED_LOOP);
                        DJIMotorSetRef(DJM3508, LIFT_MANUAL_SPEED_REF);
                    } else if (lift_manual_dir < 0) {
                        DJIMotorOuterLoop(DJM3508, SPEED_LOOP);
                        DJIMotorSetRef(DJM3508, -LIFT_MANUAL_SPEED_REF);
                    }
                }

                lift_key0_last = remote_data->KEY[0];
                lift_key1_last = remote_data->KEY[1];

            } else if ((remote_data->switch_left == 1U) &&
                       (remote_data->switch_right == 3U)) {
                level_action_started = 0U;
                lift_manual_dir = 0;

                if (release_action_started == 0U) {
                    release_action_started = 1U;
                    release_raise_target = DJM3508->measure.total_angle + LIFT_RELEASE_RAISE_DELTA;
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
                    DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                    DJIMotorSetRef(DJM3508, release_raise_target);
                }

                if (release_action_started == 1U) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
                    DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                    DJIMotorSetRef(DJM3508, release_raise_target);

                    if (fabsf(DJM3508->measure.total_angle - release_raise_target) <= LIFT_RELEASE_POS_TOL) {
                        release_action_started = 2U;
                        release_wait_start_time = HAL_GetTick();
                    }
                } else if (release_action_started == 2U) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
                    DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                    DJIMotorSetRef(DJM3508, release_raise_target);

                    if ((uint32_t)(HAL_GetTick() - release_wait_start_time) >= LIFT_RELEASE_PC2_DELAY_MS) {
                        release_action_started = 3U;
                    }
                } else if (release_action_started == 3U) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
                    DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                    DJIMotorSetRef(DJM3508, LIFT_RELEASE_DOWN_REF);

                    if (DJM3508->measure.total_angle <= LIFT_RELEASE_DOWN_TRIGGER) {
                        release_action_started = 4U;
                        release_wait_start_time = HAL_GetTick();
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
                    }
                } else if (release_action_started == 4U) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
                    DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                    DJIMotorSetRef(DJM3508, LIFT_RELEASE_FINAL_DOWN_REF);

                    if ((uint32_t)(HAL_GetTick() - release_wait_start_time) >= LIFT_RELEASE_BIGOPEN_DELAY_MS) {
                        release_action_started = 5U;
                        FeiteBigOpen();
                    }
                } else {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
                    DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                    DJIMotorSetRef(DJM3508, LIFT_RELEASE_FINAL_DOWN_REF);
                    FeiteBigOpen();
                }
            } else {
                level_action_started = 0U;
                lift_manual_dir = 0;
                release_action_started = 0U;
                release_wait_start_time = 0U;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
                DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                DJIMotorSetRef(DJM3508, LIFT_DEFAULT_ANGLE_REF);
            }
        } else {
            feite_catch_started = 0U;
            feite_over_current_count = 0U;
            feite_protected = 0U;
            level_action_started = 0U;
            lift_manual_dir = 0;
            release_action_started = 0U;
            release_wait_start_time = 0U;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
            DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
            DJIMotorSetRef(DJM3508, LIFT_DEFAULT_ANGLE_REF);
            if (dm4310_motor != NULL) {
                DMMotorSetPosVelRef(dm4310_motor, init_pos, dm4310_target_vel);
            }
            FeiteOpen();
        }
    }

    FeiteMotorControl();
}
