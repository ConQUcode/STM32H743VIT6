/**
 * @file catch.c
 * @brief 抓取机构实现 —— 飞特舵机夹具 + DJI 电机升降台
 *
 * 硬件组成:
 *   - FT_1/FT_2/FT_3: 飞特 HLS/SCS 舵机,挂 USART1 总线,控制夹具爪子的开合角度
 *   - DJM2006 (ID=1, hfdcan1): 小功率电机,驱动夹爪旋转
 *   - DJM3508 (ID=2, hfdcan1): 大功率电机,驱动升降台
 *
 * 控制流程 (CatchTask):
 *   1. LiftInit() —— 两段式机械归零:M2006 先回零,然后 M3508 靠堵转电流检测回零
 *   2. 检测遥控器 switch_left == 2 进入抓取模式
 *   3. 摇杆(rocker_r1)按下→夹取(FeiteCatch),松开→张开(FeiteOpen)
 *   4. 每周期末尾调用 FeiteMotorControl() 批量下发舵机位置
 */

#include "catch.h"
#include "daemon.h"
#include "remote.h"
#include "feite_motor.h"
#include "DJI_motor.h"
#include "cmsis_os.h"

/* 飞特舵机实例: FT_1~FT_3 控制夹具, FT_4 预留 */
static FeiteMotor_Instance *FT_1, *FT_2, *FT_3, *FT_4;
/* DJI 电机: M2006 控制夹爪旋转, M3508 控制升降 */
static DJIMotor_Instance *DJM2006, *DJM3508;
/* 两段式归零标志: M2006 先完成,然后 M3508 靠堵转检测完成 */
static int8_t is_init_2006 = 0;
static int8_t is_init_3508 = 0;

/* 外部引用: 红外传感器电平 */
int IR_sensor_level;
float control;
int a = 0;

/* -------------------------------------------------------------------------- */
/* 飞特舵机初始化 —— 4 路舵机共用 USART1 总线                                  */
/* -------------------------------------------------------------------------- */
static void FeiteMotorsInit(void)
{
    /* 默认绑定 huart1, 字节序小端, 超时 20ms */
    FeiteMotor_Bus_s *bus = FeiteMotorBusInit(NULL);

    FeiteMotor_Init_Config_s config = {
        .bus = bus,
        .model = FEITE_MODEL_HLS_SCS,
        .init_position = 0,
        .init_speed = 500,
        .init_acc = 20,
        .init_torque = 1500,
        .raw_to_deg = FEITE_DEFAULT_RAW_TO_DEG,
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    };

    config.id = 1;
    FT_1 = FeiteMotorInit(&config);

    config.id = 2;
    FT_2 = FeiteMotorInit(&config);

    config.id = 3;
    FT_3 = FeiteMotorInit(&config);

    config.id = 4;
    FT_4 = FeiteMotorInit(&config);
}

/* -------------------------------------------------------------------------- */
/* DJI 电机初始化 —— M2006(夹爪旋转) + M3508(升降) 均挂 hfdcan1                */
/* -------------------------------------------------------------------------- */
static void DJIMotorsInit(void)
{
    Motor_Init_Config_s dianji_config = {
        .can_init_config = {
            .fdcan_handle = &hfdcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 3, .Ki = 1.5, .Kd = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement | PID_DerivativeFilter |
                           PID_ErrorHandle,
                .IntegralLimit = 60000, .MaxOut = 40000, .Derivative_LPF_RC = 0.01,
            },
            .speed_PID = {
                .Kp = 3, .Ki = 0.005, .Kd = 0.004,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 10000, .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.2, .Ki = 0.0006, .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000, .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = CURRENT_AND_SPEED_FEEDFORWARD,
        },
        .motor_type = M2006,
    };
    DJM2006 = DJIMotorInit(&dianji_config);

    Motor_Init_Config_s M3508_config = {
        .can_init_config = {
            .fdcan_handle = &hfdcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, .Ki = 0.5, .Kd = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement | PID_DerivativeFilter |
                           PID_ErrorHandle,
                .IntegralLimit = 50000, .MaxOut = 50000, .Derivative_LPF_RC = 0.01,
            },
            .speed_PID = {
                .Kp = 4, .Ki = 0.025, .Kd = 0.02,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 10000, .MaxOut = 15000,
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
            .feedforward_flag = CURRENT_AND_SPEED_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
    DJM3508 = DJIMotorInit(&M3508_config);
}

/**
 * @brief 两段式机械归零
 *
 * 第一阶段 (M2006):
 *   - M3508 先抬到 15000 给 M2006 留出空间
 *   - 检测 PE11 引脚: 高电平说明机械臂在触发位 → 速度环反向驱动直到离开
 *   - PE11 变低后停止, 角度环归零
 *
 * 第二阶段 (M3508):
 *   - PE9 拉低使能
 *   - 速度环反向驱动, 靠堵转电流 (>4200mA 两次采样) 判定到达机械限位
 *   - 到达后停止, 角度环归零
 */
static void LiftInit(void)
{
    if (!is_init_2006) {
        DJIMotorSetRef(DJM3508, 15000);
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == 1) {
            /* 机械臂仍在触发位, 反向驱动使其离开 */
            DJIMotorEnable(DJM2006);
            DJIMotorOuterLoop(DJM2006, SPEED_LOOP);
            DJIMotorSetRef(DJM2006, -4000);
        } else {
            osDelay(10);
            if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == 0) {
                /* 确认已离开触发位, 停止并归零 */
                DJIMotorStop(DJM2006);
                DJIMotorReset(DJM2006);
                DJIMotorOuterLoop(DJM2006, ANGLE_LOOP);
                DJIMotorSetRef(DJM2006, 0);
                is_init_2006 = 1;
            }
        }
        return; /* M2006 归零未完成, 跳过 M3508 阶段 */
    }

    if (!is_init_3508) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
        DJIMotorEnable(DJM3508);
        DJIMotorOuterLoop(DJM3508, SPEED_LOOP);
        DJIMotorSetRef(DJM3508, -3000); /* 反向驱动, 向机械限位靠近 */

        /* 两次采样均超阈值才判定堵转, 避免电流毛刺误触发 */
        if (abs(DJM3508->measure.real_current) > 4200) {
            osDelay(5);
            if (abs(DJM3508->measure.real_current) > 4200) {
                DJIMotorStop(DJM3508);
                DJIMotorReset(DJM3508);
                DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
                DJIMotorSetRef(DJM3508, 0);
                is_init_3508 = 1;
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/* 夹具姿态控制                                                               */
/* -------------------------------------------------------------------------- */

/** 夹取姿态: 三爪向内收紧 */
static void FeiteCatch(void)
{
    FeiteMotorSetRef(FT_1, 875);
    FeiteMotorSetSpeed(FT_1, 500);
    FeiteMotorSetAcc(FT_1, 20);

    FeiteMotorSetRef(FT_2, 900);
    FeiteMotorSetSpeed(FT_2, 500);
    FeiteMotorSetAcc(FT_2, 20);

    FeiteMotorSetRef(FT_3, 590);
    FeiteMotorSetSpeed(FT_3, 500);
    FeiteMotorSetAcc(FT_3, 20);

    FeiteMotorControl();
}

/** 放下姿态: 三爪张开至中间位置 (沿用上次 speed/acc 设置) */
static void FeitePutDown(void)
{
    FeiteMotorSetRef(FT_1, 500);
    FeiteMotorSetRef(FT_2, 500);
    FeiteMotorSetRef(FT_3, 500);
    FeiteMotorControl();
}

/** 张开姿态: 三爪完全打开,释放物体 */
static void FeiteOpen(void)
{
    FeiteMotorSetRef(FT_1, 1500);
    FeiteMotorSetSpeed(FT_1, 500);
    FeiteMotorSetAcc(FT_1, 20);

    FeiteMotorSetRef(FT_2, 1500);
    FeiteMotorSetSpeed(FT_2, 500);
    FeiteMotorSetAcc(FT_2, 20);

    FeiteMotorSetRef(FT_3, 1100);
    FeiteMotorSetSpeed(FT_3, 500);
    FeiteMotorSetAcc(FT_3, 20);

    FeiteMotorControl();
}

/* -------------------------------------------------------------------------- */
/* 对外接口                                                                   */
/* -------------------------------------------------------------------------- */

void CatchInit(void)
{
    FeiteMotorsInit();
    DJIMotorsInit();
}

/**
 * @brief 抓取机构周期任务
 *
 * 由 RTOS 任务周期调用, 内部状态机包含:
 *   - 两段式归零 (LiftInit)
 *   - 遥控器 switch_left=2 时响应摇杆抓取/释放
 *   - 夹取后延时 1500ms 将 M3508 抬至 10000 完成提取动作
 */
void CatchTask(void)
{
    static uint32_t catch_start_time = 0;
    static uint8_t is_timing = 0;

    LiftInit();

    if (remote_data->switch_left == 2) {
        const uint8_t rocker_pressed = (remote_data->rocker_r1 < -500);
        DJIMotorSetRef(DJM3508, 1000);
        if (DJM3508->measure.total_angle > 900) {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
            DJIMotorSetRef(DJM2006, 9000);
        }

        if (rocker_pressed) {
            if (DJM2006->measure.total_angle > 8500) {
                if (is_timing == 0) {
                    catch_start_time = HAL_GetTick();
                    is_timing = 1;
                    FeiteCatch();
                }

                /* 夹取后延时 1500ms, 然后将升降台抬高 */
                if ((uint32_t)(HAL_GetTick() - catch_start_time) > 1500) {
                    DJIMotorSetRef(DJM3508, 10000);
                }
            }
        } else {
            /* 摇杆松开: 取消计时, 升降回位, 夹具张开 */
            is_timing = 0;
            DJIMotorSetRef(DJM3508, 1000);
            FeiteOpen();
        }
    }

    /* 每周期批量下发舵机目标位置 */
    FeiteMotorControl();
}
