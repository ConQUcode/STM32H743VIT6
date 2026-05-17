#include "arm.h"
#include "dm_motor.h"
#include "dji_motor.h"
#include "feite_motor.h"
#include "arm_kinematics.h"
#include "remote.h"
#include "bsp_dwt.h"
#include "bsp_fdcan.h"
#include "main.h"
#include "general_def.h"
#include <math.h>

/* ===================== 电机实例 ===================== */

static DMMotor_Instance *motor_j1;    /* 大臂 DM4340 */
static DJIMotor_Instance *motor_j2;   /* 小臂 M3508 */
static FeiteMotor_Instance *motor_j3; /* 末端舵机, ID=4 */

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
#define J1_MOTOR_SIGN         -1.0f

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
} arm_debug;

/* ===================== 辅助函数 ===================== */

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

    if (motor_j2 && motor_j2->feedback_initialized)
        angles->j2 = J2_HOME_ANGLE_DEG
                     + motor_j2->measure.total_angle / M3508_REDUCTION_RATIO
                     - J2_COUPLING_RATIO * angles->j1;

    if (motor_j3)
        angles->j3 = motor_j3->measure.angle_deg;
}

/**
 * @brief 把 3 个关节目标角下发给对应电机
 * @note  DM 的 ref 内部还要加回 J1 零点偏置才是物理角度
 *        J2 的 motor 命令需要加上 J1 前馈, 抵消同步带耦合
 */
static void Arm_SetAllRefs(Arm_JointAngles_t angles)
{
    DMMotorSetRef(motor_j1, J1_MOTOR_SIGN * angles.j1 + j1_zero_offset_deg);
    DJIMotorSetRef(motor_j2, (angles.j2 - J2_HOME_ANGLE_DEG
                                + J2_COUPLING_RATIO * angles.j1)
                                * M3508_REDUCTION_RATIO);

    int16_t j3_raw = (int16_t)(angles.j3 / FEITE_DEFAULT_RAW_TO_DEG);
    FeiteMotorSetRef(motor_j3, j3_raw);
    FeiteMotorSetSpeed(motor_j3, 500);
    FeiteMotorSetAcc(motor_j3, 20);
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
    /* ---- J1: DM4340 大臂电机 (MIT 模式) ---- */
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
        },
        .motor_type = DM4340,
    };

    motor_j1 = DMMotorInit(&dm_config);
    /* 先失能, 等 INIT 阶段锁定 ref=当前位置后再使能, 避免上电朝默认 0° 回零转圈 */
    DMMotorStop(motor_j1);
    DMMotorSetMITParams(motor_j1, 40.0f, 0.10f);
    DMMotorSetMITProfile(motor_j1, 40.0f, 40.0f);

    /* ---- J2: M3508 小臂电机 ---- */
    Motor_Init_Config_s j2_config = {
        .can_init_config = { .fdcan_handle = &hfdcan1, .tx_id = 3 },
        .controller_param_init_config = {
            .angle_PID   = { .Kp = 30.0f, .Ki = 0.02f, .Kd = 0.0f, .MaxOut = 30000.0f },
            .speed_PID   = { .Kp = 2.0f,  .Ki = 0.01f, .Kd = 0.0f, .MaxOut = 30000.0f },
            .current_PID = { .Kp = 1.0f,  .Ki = 0.0f,  .Kd = 0.0f, .MaxOut = 30000.0f },
        },
        .controller_setting_init_config = {
            .angle_feedback_source  = MOTOR_FEED,
            .speed_feedback_source  = MOTOR_FEED,
            .outer_loop_type        = ANGLE_LOOP,
            .close_loop_type        = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag     = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag  = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag       = FEEDFORWARD_NONE,
            .angle_mode             = MOTOR_ANGLE_MODE_TOTAL,
        },
        .motor_type = M3508,
    };
    motor_j2 = DJIMotorInit(&j2_config);
    DJIMotorEnable(motor_j2);

    /* ---- J3: 飞特舵机, USART1, ID=4 ---- */
    {
        FeiteMotor_Bus_s *bus = FeiteMotorBusInit(NULL);

        FeiteMotor_Init_Config_s j3_config = {
            .bus = bus,
            .id = 4,
            .model = FEITE_MODEL_HLS_SCS,
            .init_position = 0,
            .init_speed = 500,
            .init_acc = 20,
            .init_torque = 1500,
            .raw_to_deg = FEITE_DEFAULT_RAW_TO_DEG,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        };
        motor_j3 = FeiteMotorInit(&j3_config);
    }
}

/* ===================== 控制任务 ===================== */

void Arm_Task(void)
{
    uint8_t sw = remote_data->switch_right;

    Arm_ReadJointAngles(&current_angles);
    arm_debug.current = current_angles;
    arm_debug.sw = sw;

    switch (arm_state) {

    /* ---------------------------------------------------------------- */
    case ARM_STATE_INIT:
    /* 等 J1 首次反馈, 锁定零点和 home 位姿 */
    /* ---------------------------------------------------------------- */
        if (motor_j1 && motor_j1->feedback_initialized &&
            motor_j2 && motor_j2->feedback_initialized) {
            if (!j1_zero_inited) {
                j1_zero_offset_deg = motor_j1->measure.position_rad * RAD_2_DEGREE;
                DJIMotorReset(motor_j2);
                j1_zero_inited = 1;
                break;
            }

            /* 用新 offset 重读所有关节: J1=0, J2 自动做同步带耦合修正 */
            Arm_ReadJointAngles(&current_angles);

            home_angles = current_angles;
            Arm_Compute2LWrist(home_angles, &home_wrist_x, &home_wrist_y);
            arm_debug.home_wrist_x = home_wrist_x;
            arm_debug.home_wrist_y = home_wrist_y;

            /* 立即锁定目标=home, 防止电机使能后朝默认 0° 冲 */
            target_angles = home_angles;
            Arm_SetAllRefs(target_angles);

            DMMotorEnable(motor_j1);
            soft_start_timestamp = DWT_GetTimeline_s();
            arm_state = ARM_STATE_SOFT_START;
        }
        break;

    /* ---------------------------------------------------------------- */
    case ARM_STATE_SOFT_START:
    /* 持续发 home 1 秒让电机稳定 */
    /* ---------------------------------------------------------------- */
        target_angles = home_angles;
        Arm_SetAllRefs(target_angles);

        if (DWT_GetTimeline_s() - soft_start_timestamp >= 1.0f)
            arm_state = ARM_STATE_RUN;
        break;

    /* ---------------------------------------------------------------- */
    case ARM_STATE_RUN:
    /* 每周期按拨杆幂等写 target: sw1=home, sw2=home 腕点 +50mm, sw3=保持 */
    /* ---------------------------------------------------------------- */
        if (sw == 1) {
            target_angles = home_angles;
        } else if (sw == 2) {
            Arm_Position_t tgt;
            tgt.x   = home_wrist_x - 500.0f;
            tgt.y   = home_wrist_y + 200.0f;
            tgt.phi = 0.0f;   /* 2 连杆 IK 不使用 phi */

            arm_debug.wrist_x = tgt.x;
            arm_debug.wrist_y = tgt.y;
            arm_debug.wrist_r = sqrtf(tgt.x * tgt.x + tgt.y * tgt.y);

            Arm_JointAngles_t ik;
            uint8_t ik_ret = Arm_IK(tgt, &ik, ARM_ELBOW_DOWN);
            arm_debug.last_ik = ik;
            arm_debug.last_ik_ret = ik_ret;

            uint8_t limits_ok = ik_ret ? Arm_CheckJointLimits(ik) : 0;
            arm_debug.last_limits_ok = limits_ok;

            if (ik_ret && limits_ok) {
                target_angles.j1 = ik.j1;
                target_angles.j2 = ik.j2;
                target_angles.j3 = home_angles.j3;   /* J3 保持 home */
                arm_debug.ik_ok_count++;
            } else {
                arm_debug.ik_fail_count++;
                /* IK 失败: 保持上次 target 不变 */
            }
        }
        /* sw == 3 (或其它): 保持上次 target */

        Arm_SetAllRefs(target_angles);
        arm_debug.target = target_angles;
        break;
    }

    /* DM 电机控制发送 (DJI/舵机由独立 FreeRTOS 任务调用) */
    DMMotorControl();
}
