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
#include <math.h>

#define AIR_MODULE_1_GPIO_PORT GPIOC
#define AIR_MODULE_1_GPIO_PIN  GPIO_PIN_8
#define AIR_MODULE_2_GPIO_PORT GPIOD
#define AIR_MODULE_2_GPIO_PIN  GPIO_PIN_3
#define AIR_MODULE_ON_LEVEL    GPIO_PIN_SET
#define AIR_MODULE_OFF_LEVEL   GPIO_PIN_RESET
#define AIR_REMOTE_SWITCH_ON   2U

/* ===================== 遥控器任务映射说明 =====================
 *
 * 当前 ARM 任务使用 RadioMaster Boxer / ELRS 语义化字段:
 *   - remote_boxer.sb           : 左侧三档拨杆, 作为 ARM 子任务选择
 *   - remote_boxer.sc           : 右侧三档拨杆, 作为当前子任务下的预设档位选择
 *   - remote_boxer.right_y      : 右摇杆 Y 轴, 作为当前关节的手动微调输入
 *   - remote_boxer.sa           : 1 开启气泵 1 (PC8), 2 关闭
 *   - remote_boxer.sd           : 1 开启气泵 2 (PD3), 2 关闭
 *
 * 左拨杆任务分配:
 *   - 左拨杆 1 / 其他值: 不切换新预设, J1/J2 保持当前目标
 *   - 左拨杆 2: J1 任务
 *       右拨杆 1/2/3 -> Arm_ApplyPreset(), 设置 3/4 号幻尔舵机预设和 J1 预设
 *       右摇杆 Y     -> 在当前 J1 目标基础上微调
 *   - 左拨杆 3: J2 任务
 *       右拨杆 1/2/3 -> Arm_ApplyPreset2(), 设置 1/2 号幻尔舵机预设和 J2 预设
 *       右摇杆 Y     -> 在当前 J2 目标基础上微调
 *
 * 与 CatchTask 的分工:
 *   - SB=1: CatchTask 工作区间
 *   - SB=2: ARM J1 工作区间
 *   - SB=3: ARM J2 工作区间
 *   SC=1/2/3 在当前工作区间内选择预设, 因此和 catch 不直接冲突。
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
#define SERVO3_PRESET_1_POS  265U
#define SERVO4_PRESET_1_POS  677U
#define SERVO3_PRESET_2_POS  465U
#define SERVO4_PRESET_2_POS  300U
#define SERVO3_PRESET_3_POS  720U
#define SERVO4_PRESET_3_POS  300U
#define SERVO_MOVE_TIME_MS   100U
#define J1_INIT_POS_RAD      (0.0f)
#define J1_RC_STEP_RAD       0.03f
#define J1_MAX_VEL_RAD_S     0.8f
#define J1_PRESET_1_DEG      (1.2f * RAD_2_DEGREE)
#define J1_PRESET_2_DEG      (1.2f * RAD_2_DEGREE)
#define J1_PRESET_3_DEG      (3.7f * RAD_2_DEGREE)
#define H1_INIT_POS          390U
#define H2_INIT_POS          440U
#define H1_PRESET_1_POS      390U
#define H2_PRESET_1_POS      170U
#define H1_PRESET_2_POS      780U
#define H2_PRESET_2_POS      230U
#define H1_PRESET_3_POS      780U
#define H2_PRESET_3_POS      78U
#define J2_INIT_POS_RAD      (0.0f)
#define J2_MAX_VEL_RAD_S     0.8f
#define J2_RC_STEP_RAD       0.03f
#define J2_PRESET_1_DEG      (-1.0f * RAD_2_DEGREE)
#define J2_PRESET_2_DEG      (-1.5f * RAD_2_DEGREE)
#define J2_PRESET_3_DEG      (-2.0f * RAD_2_DEGREE)
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
static uint8_t arm_last_left_switch = 0U;
static uint8_t arm_last_right_switch = 0U;

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
    uint8_t air_pc8_on;
    uint8_t air_pd3_on;
} arm_debug;

/* ===================== 辅助函数 ===================== */

/* Air outputs: high = on, low = off. */
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

static uint8_t ArmRemoteSwitchValid(uint8_t value)
{
    return ((value >= 1U) && (value <= 3U)) ? 1U : 0U;
}

static uint8_t ArmRemoteLeftSwitch(void)
{
    return remote_boxer.sb;
}

static uint8_t ArmRemoteRightSwitch(void)
{
    return remote_boxer.sc;
}

static float ArmRemoteRightY(void)
{
    return (float)remote_boxer.right_y;
}

static void Arm_ProcessAirKeys(void)
{
    /* SA 控制气泵 1 (PC8): 1 开启, 2 关闭 */
    if (remote_boxer.sa == 1U) {
        Arm_AirModule1Set(1U);
    } else if (remote_boxer.sa == 2U) {
        Arm_AirModule1Set(0U);
    }

    /* SD 控制气泵 2 (PD3): 1 开启, 2 关闭 */
    if (remote_boxer.sd == 1U) {
        Arm_AirModule2Set(1U);
    } else if (remote_boxer.sd == 2U) {
        Arm_AirModule2Set(0U);
    }
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

static void Arm_SetServoTargets2(uint16_t h1_pos, uint16_t h2_pos)
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

static float Arm_GetPresetJ1Deg(uint8_t right_sw)
{
    /* 左拨杆=2 时, 右拨杆 1/2/3 对应 J1 三个预设角。 */
    switch (right_sw) {
        case 1U: return J1_PRESET_1_DEG;
        case 2U: return J1_PRESET_2_DEG;
        case 3U: return J1_PRESET_3_DEG;
        default: return target_angles.j1;
    }
}

static void Arm_ApplyPreset(uint8_t right_sw)
{
    /*
     * 左拨杆=3 的 J1 任务页:
     *   右拨杆 1/2/3 选择 3/4 号幻尔舵机预设, 并同步设置 J1 预设角。
     */
    switch (right_sw) {
        case 1U: Arm_SetServoTargets2(H1_PRESET_1_POS, H2_PRESET_1_POS);  break;
        case 2U: Arm_SetServoTargets2(H1_PRESET_2_POS, H2_PRESET_2_POS);  break;
        case 3U: Arm_SetServoTargets2(H1_PRESET_3_POS, 215);  
			           Arm_SetServoTargets(765, 677);
			           target_j2_deg = -3.7* RAD_2_DEGREE;
			break;//这里是两个机械臂配合吸取，j2长机械臂预设在这，本来不该这样写的，心烦不改了吧
        default:        Arm_SetServoTargets2(H1_INIT_POS, H2_INIT_POS);  return;
    }
    target_angles.j1 = Arm_GetPresetJ1Deg(right_sw);
}

static void Arm_ApplyPreset2(uint8_t right_sw)
{
    /*
     * 左拨杆=3 的 J2 任务页:
     *   右拨杆 1/2/3 选择 1/2/3/4 号幻尔舵机预设, 并同步设置 J2 预设角。
     */
    switch (right_sw) {
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
    uint8_t left_sw = ArmRemoteLeftSwitch();
    uint8_t right_sw = ArmRemoteRightSwitch();

    /* 处理气路控制 (SA/SD) */
    Arm_ProcessAirKeys();

    Arm_ReadJointAngles(&current_angles);
    arm_debug.current = current_angles;
    arm_debug.sw = right_sw;

    if (motor_j1 && motor_j1->feedback_initialized && !j1_zero_inited) {
        j1_zero_offset_deg = 0.0f;
        j1_zero_inited = 1;
        target_angles.j1 = J1_INIT_POS_RAD * RAD_2_DEGREE; // 逻辑角度 2.7 rad
        target_angles.j2 = current_angles.j2;
        target_angles.j3 = current_angles.j3;
    }

    /*
     * 左拨杆=2: J1 控制任务
     *   - 第一次进入该任务, 或右拨杆档位变化时, 按右拨杆 1/2/3 应用 J1 + 3/4 号舵机预设。
     *   - 右摇杆 Y(remote_boxer.right_y) 作为 J1 微调输入。
     *   - J2 保持当前 target_j2_deg。
     */
    if (left_sw == 2U) {
        if ((arm_last_left_switch != 2U) || (arm_last_right_switch != right_sw)) Arm_ApplyPreset(right_sw);
        {
            float j1_step_rad = ArmRemoteRightY() / 660.0f * J1_RC_STEP_RAD;
            if (fabsf(j1_step_rad) < 0.0002f) j1_step_rad = 0.0f;
            target_angles.j1 += j1_step_rad * RAD_2_DEGREE;
            if (target_angles.j1 > 400.0f) target_angles.j1 = 400.0f;
            if (target_angles.j1 < -400.0f) target_angles.j1 = -400.0f;
        }
        DMMotorSetPosVelRef(motor_j1, J1_MOTOR_SIGN * target_angles.j1 * DEGREE_2_RAD, J1_MAX_VEL_RAD_S);
        if (motor_j2) DMMotorSetPosVelRef(motor_j2, J2_MOTOR_SIGN * target_j2_deg * DEGREE_2_RAD, J2_MAX_VEL_RAD_S);
    /*
     * 左拨杆=3: J2 控制任务
     *   - 第一次进入该任务, 或右拨杆档位变化时, 按右拨杆 1/2/3 应用 J2 + 1/2 号舵机预设。
     *   - 右摇杆 Y(remote_boxer.right_y) 作为 J2 微调输入。
     *   - J1 保持当前 target_angles.j1。
     */
    } else if (left_sw == 3U) {
        if ((arm_last_left_switch != 3U) || (arm_last_right_switch != right_sw)) Arm_ApplyPreset2(right_sw);
        {
            float j2_step_rad = ArmRemoteRightY() / 660.0f * J2_RC_STEP_RAD;
            if (fabsf(j2_step_rad) < 0.0002f) j2_step_rad = 0.0f;
            target_j2_deg += j2_step_rad * RAD_2_DEGREE;
            if (target_j2_deg > 180.0f) target_j2_deg = 180.0f;
            if (target_j2_deg < -250.0f) target_j2_deg = -250.0f;
        }
        DMMotorSetPosVelRef(motor_j2, J2_MOTOR_SIGN * target_j2_deg * DEGREE_2_RAD, J2_MAX_VEL_RAD_S);
        DMMotorSetPosVelRef(motor_j1, J1_MOTOR_SIGN * target_angles.j1 * DEGREE_2_RAD, J1_MAX_VEL_RAD_S);
    } else {
        /*
         * 左拨杆=1 或遥控器数据不可用:
         *   不应用新预设, 不响应摇杆微调, 只持续保持当前 J1/J2 目标。
         */
        DMMotorSetPosVelRef(motor_j1, J1_MOTOR_SIGN * target_angles.j1 * DEGREE_2_RAD, J1_MAX_VEL_RAD_S);
        if (motor_j2) DMMotorSetPosVelRef(motor_j2, J2_MOTOR_SIGN * target_j2_deg * DEGREE_2_RAD, J2_MAX_VEL_RAD_S);
    }

    arm_last_left_switch = left_sw;
    arm_last_right_switch = right_sw;
    arm_debug.target = target_angles;


    HEMotorControl();
}
