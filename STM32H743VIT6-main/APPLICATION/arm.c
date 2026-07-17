#include "arm.h"
#include "dm_motor.h"
#include "dji_motor.h"
#include "feite_motor.h"
#include "HEmotor.h"
#include "arm_kinematics.h"
#include "remote.h"
#include "bsp_dwt.h"
#include "bsp_fdcan.h"
#include "main.h"
#include "general_def.h"
#include "usb.h"
#include "remote_logic_profile.h"
#include <math.h>

#define AIR_MODULE_1_GPIO_PORT GPIOD
#define AIR_MODULE_1_GPIO_PIN  GPIO_PIN_2
#define AIR_MODULE_2_GPIO_PORT GPIOD
#define AIR_MODULE_2_GPIO_PIN  GPIO_PIN_3
#define AIR_MODULE_ON_LEVEL    GPIO_PIN_RESET
#define AIR_MODULE_OFF_LEVEL   GPIO_PIN_SET
#define AIR_REMOTE_SWITCH_ON   2U

/* ===================== 遥控器任务映射说明 =====================
 *
 * SB=1: catch 区域。SD 只触发 USB 屏幕下一张，ARM 气泵关闭。
 * SB=2: arm 区域。进入时 six_pos 必须先回到 1 才解锁。
 *
 * ARM 解锁后:
 *   - six_pos=1: 空档，保持当前 J1 目标。
 *   - six_pos=2/3/4: J1 预设 1/2/3 档，并同步 H1/H2 舵机预设。
 *   - six_pos=5: J1 达妙电机失能；切到其他档位后重新使能。
 *   - six_pos=6: 空档，保持当前 J1 目标。
 *
 * ARM 模式不再使用 SB/SC 选任务，ALT profile 不在主任务中下发 J2 新目标。
 * 小机械臂气泵遥控控制暂时移除并默认关闭。
 */

/* ===================== 电机实例 ===================== */

static DMMotor_Instance *motor_j1;    /* 大臂 DM4340 */
static DMMotor_Instance *motor_j2;    /* 新增达妙电机 */
static HEMotor_Instance *motor_j3;    /* 4号幻尔舵机, USART3 */
static HEMotor_Instance *motor_he;    /* 3号幻尔舵机, USART3 */
static HEMotor_Instance *motor_h1;    /* 1号幻尔舵机, USART6 */
static HEMotor_Instance *motor_h2;    /* 2号幻尔舵机, USART6 */

/* 舵机与达妙控制参数 */
#define SERVO3_INIT_POS      470U
#define SERVO4_INIT_POS      677U
#define SERVO3_PRESET_1_POS  85U
#define SERVO4_PRESET_1_POS  685U
#define SERVO3_PRESET_2_POS  470U
#define SERVO4_PRESET_2_POS  290U
#define SERVO3_PRESET_3_POS  470U
#define SERVO4_PRESET_3_POS  290U
#define SERVO_MOVE_TIME_MS   100U
#define J1_INIT_POS_RAD      (0.0f)
#define J1_RC_STEP_RAD       0.03f
#define J1_MAX_VEL_RAD_S     1.2f
#define J1_PRESET_1_DEG      (1.2f * RAD_2_DEGREE)
#define J1_PRESET_2_DEG      (1.2f * RAD_2_DEGREE)
#define J1_PRESET_3_DEG      (3.855f * RAD_2_DEGREE)
#define H1_INIT_POS          190U
#define H2_INIT_POS          440U
#define H1_PRESET_1_POS      390U
#define H2_PRESET_1_POS      170U
#define H1_PRESET_2_POS      480U
#define H2_PRESET_2_POS      230U
#define H1_PRESET_3_POS      480U
#define H2_PRESET_3_POS      140U
#define ARM_ACTION1_DM3_POS_RAD   (3.1f)
#define ARM_ACTION1_SERVO2_POS    440U
#define ARM_ACTION2_DM2_POS_RAD   (-3.1f)
#define ARM_ACTION2_SERVO3_POS    470U
#define ARM_ACTION2_SERVO4_POS    677U
#define J2_INIT_POS_RAD      (-3.0f)
#define J2_MAX_VEL_RAD_S     0.8f
#define J2_RC_STEP_RAD       0.03f
#define J2_PRESET_1_DEG      (-1.3f * RAD_2_DEGREE)
#define J2_PRESET_2_DEG      (-1.3f * RAD_2_DEGREE)
#define J2_PRESET_3_DEG      (-1.7f * RAD_2_DEGREE)
#define J2_MOTOR_SIGN        1.0f

/* J2 减速比 & 同步带耦合系数:
 * M3508 内置 1:19 减速器, 编码器测量的是转子角度 (减速前),
 * total_angle 是转子角度, 需 /19 才得到输出轴角度.
 * 3508 固定在底座上, 通过同步带驱动小臂相对底座的绝对角:
 *   j2_关节 = J2_HOME_ANGLE_DEG + motor_rotor_angle / 19 - J2_COUPLING_RATIO * j1
 *   motor_rotor_ref = (j2_关节 - J2_HOME_ANGLE_DEG + J2_COUPLING_RATIO * j1) * 19
 * home 时大臂水平向前, 小臂反向水平折叠, 所以小臂相对大臂角为 180°. */
#define M3508_REDUCTION_RATIO 19.0f
#define J2_COUPLING_RATIO     1.0f
#define J2_HOME_ANGLE_DEG     180.0f
#define J1_MOTOR_SIGN         1.0f

/* ===================== 状态机 ===================== */

typedef enum {
    ARM_STATE_INIT = 0,
    ARM_STATE_SOFT_START,
    ARM_STATE_RUN,
} Arm_State_e;

static volatile Arm_State_e arm_state = ARM_STATE_INIT;
static float soft_start_timestamp = 0.0f;

/* ===================== 零点 & 基准位姿 ===================== */

static float j1_zero_offset_deg = 0.0f;   /* 上电时锁定的 J1 零点 (度) */
static uint8_t j1_zero_inited = 0;

static Arm_JointAngles_t home_angles;     /* 上电锁定的 home 关节角 */
static float home_wrist_x = 0.0f;         /* home 时 L2 末端 (腕点) X 坐标 */
static float home_wrist_y = 0.0f;         /* home 时 L2 末端 (腕点) Y 坐标 */

/* ===================== 当前 & 目标 ===================== */

static Arm_JointAngles_t current_angles;  /* 本周期读到的当前角度 */
static Arm_JointAngles_t target_angles;   /* 下发给电机的目标角度 */
static float target_j2_deg = J2_INIT_POS_RAD * RAD_2_DEGREE;
static uint8_t arm_mode_was_active = 0U;
static uint8_t arm_six_pos_unlocked = 0U;
static uint8_t arm_last_six_pos = 0U;
static uint8_t arm_j1_remote_stopped = 0U;
static uint8_t catch_last_sd_switch = 0U;
static uint8_t arm_last_sd_switch = 0U;

/* ===================== 调试观测 ===================== */

static volatile struct {
    Arm_JointAngles_t current;   /* 当前读到的角度 */
    Arm_JointAngles_t target;    /* 本周期下发的目标 */
    Arm_JointAngles_t last_ik;   /* 最近一次 IK 解算结果 (无论成败) */
    float home_wrist_x;          /* home 时记录的腕点 X */
    float home_wrist_y;          /* home 时记录的腕点 Y */
    float wrist_x;               /* sw==2 时解算出的腕点目标 X */
    float wrist_y;               /* sw==2 时解算出的腕点目标 Y */
    float wrist_r;               /* 腕点目标距原点距离 (用于判断是否超出 IK 工作空间) */
    uint8_t last_ik_ret;         /* 最近一次 Arm_IK 返回值 (0=超工作空间) */
    uint8_t last_limits_ok;      /* 最近一次 CheckJointLimits 结果 */
    uint8_t sw;                  /* 本周期读到的拨杆档位 */
    uint16_t ik_ok_count;        /* IK 成功次数 */
    uint16_t ik_fail_count;      /* IK 失败次数 (含超限) */
    float he_cmd_pos;            /* 当前 HE 发送值 */
    float he_follow_deg;         /* HE 跟随使用的 J1 角度 */
    uint8_t mode;
    uint8_t six_pos;
    uint8_t six_pos_unlocked;
    uint8_t last_six_pos;
    uint8_t j1_remote_stopped;
    uint8_t air_pc8_on;
    uint8_t air_pd3_on;
} arm_debug;

/* ===================== 辅助函数 ===================== */

/* Air outputs: low = on, high = off. */
static void Arm_AirOutputSet(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, uint8_t enable)
{
    uint8_t on = (enable != 0U) ? 1U : 0U;

    HAL_GPIO_WritePin(gpio_port,
                      gpio_pin,
                      on ? AIR_MODULE_ON_LEVEL : AIR_MODULE_OFF_LEVEL);
}

static void Arm_AirModule1Set(uint8_t enable)
{
    uint8_t on = (enable != 0U) ? 1U : 0U;

    Arm_AirOutputSet(AIR_MODULE_1_GPIO_PORT, AIR_MODULE_1_GPIO_PIN, on);
    arm_debug.air_pc8_on = on;
}

static void Arm_AirModule2Set(uint8_t enable)
{
    uint8_t on = (enable != 0U) ? 1U : 0U;

    Arm_AirOutputSet(AIR_MODULE_2_GPIO_PORT, AIR_MODULE_2_GPIO_PIN, on);
    arm_debug.air_pd3_on = on;
}

static uint8_t ArmRemoteSixPos(void)
{
    return remote_boxer.six_pos;
}

static uint8_t ArmRemoteTaskModeIsCatch(void)
{
    return (remote_boxer.sb == 1U) ? 1U : 0U;
}

static uint8_t ArmRemoteTaskModeIsArm(void)
{
    return (remote_boxer.sb == 2U) ? 1U : 0U;
}

static void Arm_ProcessAirKeys(uint8_t arm_unlocked)
{
    uint8_t current_sd = remote_boxer.sd;

#if REMOTE_LOGIC_PROFILE == REMOTE_LOGIC_PROFILE_ALT
    /*
     * Profile ALT assumes the J1 air pump is wired to AirModule1 (PD2).
     * AirModule2 is the original J2/arm pump path and is held off here.
     */
    if (current_sd == 1U) {
        Arm_AirModule1Set(0U);
    } else if (current_sd == 2U) {
        Arm_AirModule1Set(1U);
    }

    if (ArmRemoteTaskModeIsCatch() != 0U) {
        Arm_AirModule2Set(0U);

        if ((catch_last_sd_switch == 1U) && (current_sd == 2U)) {
            if (!USB_ScreenIsBusy()) {
                USB_ScreenClearAckFlag();
                (void)USB_ScreenSendNext();
            }
        }

        catch_last_sd_switch = current_sd;
        arm_last_sd_switch = current_sd;
        return;
    }

    if (ArmRemoteTaskModeIsArm() != 0U) {
        Arm_AirModule2Set(0U);

        arm_last_sd_switch = current_sd;
        catch_last_sd_switch = current_sd;
        return;
    }

    Arm_AirModule2Set(0U);
    arm_last_sd_switch = current_sd;
    catch_last_sd_switch = current_sd;
#else
    Arm_AirModule1Set(0U);

    if (ArmRemoteTaskModeIsCatch() != 0U) {
        Arm_AirModule2Set(0U);

        if ((catch_last_sd_switch == 1U) && (current_sd == 2U)) {
            if (!USB_ScreenIsBusy()) {
                USB_ScreenClearAckFlag();
                (void)USB_ScreenSendNext();
            }
        }

        catch_last_sd_switch = current_sd;
        arm_last_sd_switch = current_sd;
        return;
    }

    if ((ArmRemoteTaskModeIsArm() != 0U) && (arm_unlocked != 0U)) {
        if (current_sd == 1U) {
            Arm_AirModule2Set(0U);
        } else if (current_sd == 2U) {
            Arm_AirModule2Set(1U);
        }

        arm_last_sd_switch = current_sd;
        catch_last_sd_switch = current_sd;
        return;
    }

    Arm_AirModule2Set(0U);
    arm_last_sd_switch = current_sd;
    catch_last_sd_switch = current_sd;
#endif
}

static void Arm_SetServoTargets(uint16_t servo3_pos, uint16_t servo4_pos)
{
    if (motor_he != NULL) {
        motor_he->ref.position = servo3_pos;
        motor_he->ref.time = SERVO_MOVE_TIME_MS;
    }
    if (motor_j3 != NULL) {
        motor_j3->ref.position = servo4_pos;
        motor_j3->ref.time = SERVO_MOVE_TIME_MS;
    }
}

void Arm_ActionPreset2(void)
{
    target_j2_deg = ARM_ACTION2_DM2_POS_RAD * RAD_2_DEGREE;
    if (motor_j2 != NULL) {
        DMMotorSetPosVelRef(motor_j2,
                            J2_MOTOR_SIGN * ARM_ACTION2_DM2_POS_RAD,
                            J2_MAX_VEL_RAD_S);
    }

    Arm_SetServoTargets(ARM_ACTION2_SERVO3_POS, ARM_ACTION2_SERVO4_POS);
}

#if REMOTE_LOGIC_PROFILE == REMOTE_LOGIC_PROFILE_CURRENT
static void Arm_ApplyFormerJ1Sc3J2Part(void)
{
    Arm_SetServoTargets(765U, 677U);
    target_j2_deg = -3.7f * RAD_2_DEGREE;
}

static void Arm_ApplyPreset2(uint8_t preset_index)
{
    /*
     * 左拨杆=3 的 J2 任务页:
     *   右拨杆 1/2/3 选择 1/2/3/4 号幻尔舵机预设, 并同步设置 J2 预设角。
     */
    switch (preset_index) {
        case 1U: 
          
            Arm_SetServoTargets(SERVO3_PRESET_1_POS, SERVO4_PRESET_1_POS);
            target_j2_deg = J2_PRESET_1_DEG; 
            break;
        case 2U: 
        
            Arm_SetServoTargets(SERVO3_PRESET_2_POS, SERVO4_PRESET_2_POS);
            target_j2_deg = J2_PRESET_2_DEG; 
            break;
        case 3U: 
            
            Arm_SetServoTargets(SERVO3_PRESET_3_POS, SERVO4_PRESET_3_POS);
            target_j2_deg = J2_PRESET_3_DEG; 
            break;
        default: 
     
            Arm_SetServoTargets(SERVO3_INIT_POS, SERVO4_INIT_POS);
            target_j2_deg = J2_INIT_POS_RAD * RAD_2_DEGREE; 
            break;
    }
}
#endif

#if REMOTE_LOGIC_PROFILE == REMOTE_LOGIC_PROFILE_ALT
static void Arm_SetJ1ServoTargets(uint16_t h1_pos, uint16_t h2_pos)
{
    if (motor_h1 != NULL) {
        motor_h1->ref.position = h1_pos;
        motor_h1->ref.time = SERVO_MOVE_TIME_MS;
    }
    if (motor_h2 != NULL) {
        motor_h2->ref.position = h2_pos;
        motor_h2->ref.time = SERVO_MOVE_TIME_MS;
    }
}

static void Arm_ApplyJ1OnlyPreset(uint8_t preset_index)
{
    switch (preset_index) {
        case 1U:
            target_angles.j1 = J1_PRESET_1_DEG;
            Arm_SetJ1ServoTargets(H1_PRESET_1_POS, H2_PRESET_1_POS);
            break;

        case 2U:
            target_angles.j1 = J1_PRESET_2_DEG;
            Arm_SetJ1ServoTargets(H1_PRESET_2_POS, H2_PRESET_2_POS);
            break;

        case 3U:
            target_angles.j1 = J1_PRESET_3_DEG;
            Arm_SetJ1ServoTargets(H1_PRESET_3_POS, H2_PRESET_3_POS);
            break;

        default:
            break;
    }
}
#endif

/**
 * @brief 读取 3 个关节的当前角度
 * @note  J1 减去上电锁定的零点偏置, 使 home 位置对应 J1=0
 *        J2 减去 J1 耦合分量, 还原成"小臂相对大臂"的关节角
 */
static void Arm_ReadJointAngles(Arm_JointAngles_t *angles)
{
    if (motor_j1 && motor_j1->feedback_initialized)
        angles->j1 = J1_MOTOR_SIGN *
                     (motor_j1->measure.position_rad * RAD_2_DEGREE - j1_zero_offset_deg);

    /* HE 舵机为开环控制，目前不读取反馈角度 */
    angles->j3 = 0.0f;
}


/**
 * @brief 2 连杆正运动学: (j1, j2) → 腕点 (L2 末端) 坐标
 * @note  用于记录 home 腕点位置; 刻意不调 Arm_FK (3 连杆) 保持与 2 连杆 IK 一致
 */
static void Arm_Compute2LWrist(Arm_JointAngles_t angles, float *x, float *y)
{
    float a1  = angles.j1 * DEGREE_2_RAD;
    float a12 = a1 + angles.j2 * DEGREE_2_RAD;
    *x = ARM_L1 * cosf(a1) + ARM_L2 * cosf(a12);
    *y = ARM_L1 * sinf(a1) + ARM_L2 * sinf(a12);
}

/* ===================== 初始化 ===================== */

void Arm_Init(void)
{
    Arm_AirModule1Set(0U);
    Arm_AirModule2Set(0U);

    /* ---- J1: DM4340 大臂电机 (模式) ---- */
    Motor_Init_Config_s dm_config = {
        .can_init_config = {
            .fdcan_handle = &hfdcan1,
            .tx_id = 3,
            .rx_id = 0x13,
        },
        .controller_setting_init_config = {
            .motor_reverse_flag     = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag  = FEEDBACK_DIRECTION_NORMAL,
            .angle_mode             = MOTOR_ANGLE_MODE_SINGLE_TURN,
            .close_loop_type        = OPEN_LOOP,
        },
        .motor_type = DM4340,
    };

    motor_j1 = DMMotorInit(&dm_config);

		DMMotorSetControlMode(motor_j1, DM_MODE_POS_VEL );
    DMMotorEnable(motor_j1);

    target_angles.j1 = J1_INIT_POS_RAD * RAD_2_DEGREE; // 逻辑角度保持为正
    DMMotorSetPosVelRef(motor_j1, J1_MOTOR_SIGN * J1_INIT_POS_RAD, J1_MAX_VEL_RAD_S); // 物理角度 = 方向系数 * 2.7 rad

    /* ---- 4号幻尔舵机 ---- */
    {
        HEMotor_Init_Config_s j3_config = {
            .huart = &huart3,
            .motor_config = { .id = 4, .cmd = SERVO_MOVE_TIME_WRITE },
            .motor_ref = { .position = SERVO4_INIT_POS, .time = SERVO_MOVE_TIME_MS, .stop_flag = HE_ENABLED },
        };
        motor_j3 = HEMotorInit(&j3_config);
    }

    /* ---- 3号幻尔舵机 ---- */
    {
        HEMotor_Init_Config_s he_config = {
            .huart = &huart3,
            .motor_config = { .id = 3, .cmd = SERVO_MOVE_TIME_WRITE },
            .motor_ref = { .position = SERVO3_INIT_POS, .time = SERVO_MOVE_TIME_MS, .stop_flag = HE_ENABLED },
        };
        motor_he = HEMotorInit(&he_config);
    }

    /* ---- 新增达妙电机 ---- */
    {
        Motor_Init_Config_s dm2_config = dm_config;
        dm2_config.can_init_config.tx_id = 2;
        dm2_config.can_init_config.rx_id = 0x12;
        motor_j2 = DMMotorInit(&dm2_config);
        DMMotorSetControlMode(motor_j2, DM_MODE_POS_VEL);
        DMMotorEnable(motor_j2);
        target_j2_deg = J2_INIT_POS_RAD * RAD_2_DEGREE;
        DMMotorSetPosVelRef(motor_j2, J2_MOTOR_SIGN * J2_INIT_POS_RAD, J2_MAX_VEL_RAD_S);
    }

    { HEMotor_Init_Config_s cfg = {
		.huart = &huart3,
    .motor_config = { .id = 1, .cmd = SERVO_MOVE_TIME_WRITE },
		.motor_ref = { .position = H1_INIT_POS, .time = SERVO_MOVE_TIME_MS, .stop_flag = HE_ENABLED }
		};
		motor_h1 = HEMotorInit(&cfg); }//1号没有用，这里偷懒不删除了
    { HEMotor_Init_Config_s cfg = {
		.huart = &huart3,
    .motor_config = { .id = 2, .cmd = SERVO_MOVE_TIME_WRITE },
		.motor_ref = { .position = H2_INIT_POS, .time = SERVO_MOVE_TIME_MS, .stop_flag = HE_ENABLED } };
		motor_h2 = HEMotorInit(&cfg);
		}
}

/* ===================== 控制任务 ===================== */

void Arm_Task(void)
{
    uint8_t six_pos = ArmRemoteSixPos();
    uint8_t arm_mode_active = ArmRemoteTaskModeIsArm();

    if (arm_mode_active == 0U) {
        arm_debug.mode = remote_boxer.sb;
        arm_debug.six_pos = six_pos;
        arm_debug.six_pos_unlocked = arm_six_pos_unlocked;
        arm_debug.last_six_pos = arm_last_six_pos;
        arm_debug.j1_remote_stopped = arm_j1_remote_stopped;
        HEMotorControl();
        return;
    }

    if (arm_mode_was_active == 0U) {
        arm_mode_was_active = 1U;
        arm_six_pos_unlocked = (six_pos == 1U) ? 1U : 0U;
    } else if (six_pos == 1U) {
        arm_six_pos_unlocked = 1U;
    }

    Arm_ProcessAirKeys(arm_six_pos_unlocked);

    Arm_ReadJointAngles(&current_angles);
    arm_debug.current = current_angles;
    arm_debug.sw = six_pos;
    arm_debug.mode = 2U;
    arm_debug.six_pos = six_pos;
    arm_debug.six_pos_unlocked = arm_six_pos_unlocked;
    arm_debug.last_six_pos = arm_last_six_pos;

    if (motor_j1 && motor_j1->feedback_initialized && !j1_zero_inited) {
        j1_zero_offset_deg = 0.0f;
        j1_zero_inited = 1;
        target_angles.j1 = J1_INIT_POS_RAD * RAD_2_DEGREE; // 逻辑角度 2.7 rad
        target_angles.j2 = current_angles.j2;
        target_angles.j3 = current_angles.j3;
    }

#if REMOTE_LOGIC_PROFILE == REMOTE_LOGIC_PROFILE_ALT
    if ((arm_j1_remote_stopped != 0U) && (six_pos != 5U)) {
        DMMotorEnable(motor_j1);
        arm_j1_remote_stopped = 0U;
    }

    if (arm_six_pos_unlocked != 0U) {
        switch (six_pos) {
            case 2U:
                Arm_ApplyJ1OnlyPreset(1U);
                break;

            case 3U:
                Arm_ApplyJ1OnlyPreset(2U);
                break;

            case 4U:
                Arm_ApplyJ1OnlyPreset(3U);
                break;

            case 5U:
                if (arm_j1_remote_stopped == 0U) {
                    DMMotorStop(motor_j1);
                    arm_j1_remote_stopped = 1U;
                }
                break;

            default:
                break;
        }
    }

    arm_debug.j1_remote_stopped = arm_j1_remote_stopped;

    if ((motor_j1 != NULL) && (arm_j1_remote_stopped == 0U)) {
        DMMotorSetPosVelRef(motor_j1,
                            J1_MOTOR_SIGN * target_angles.j1 * DEGREE_2_RAD,
                            J1_MAX_VEL_RAD_S);
    }
#else
    if (arm_six_pos_unlocked != 0U) {
        switch (six_pos) {
            case 2U:
                Arm_ApplyPreset2(1U);
                break;

            case 3U:
                Arm_ApplyPreset2(2U);
                break;

            case 4U:
                Arm_ApplyPreset2(3U);
                break;

            case 5U:
                Arm_ActionPreset2();
                break;

            case 6U:
                Arm_ApplyFormerJ1Sc3J2Part();
                break;

            default:
                break;
        }
    }

    if (motor_j2 != NULL) {
        DMMotorSetPosVelRef(motor_j2,
                            J2_MOTOR_SIGN * target_j2_deg * DEGREE_2_RAD,
                            J2_MAX_VEL_RAD_S);
    }

    target_angles.j2 = target_j2_deg;
#endif
    arm_last_six_pos = six_pos;
    arm_debug.target = target_angles;

    HEMotorControl();
}
