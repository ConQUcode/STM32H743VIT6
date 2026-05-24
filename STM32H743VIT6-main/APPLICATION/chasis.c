/**
 * @file chassis.c
 * @author 
 * @brief 底盘控制实现，包含电机初始化、IMU 处理和运动学分解
 * @version 0.1
 * @date 2024-xx-xx
 */

#include "chassis.h"
#include "DJI_motor.h"
#include "attitude_ekf.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "robot_def.h"
#include <math.h>

/* M3508 行走轮低速死区:
 * 舵轮运动学算出的 vt_* 小于该值时认为是停车, 直接给行走轮 0 速.
 * 原来 300 附近容易在 Stop/Enable 之间反复切换, 微小速度时会抖,
 * 所以这里降到 80, 让低速指令更连续。 */
#define CHASSIS_ID1_M3508_SPEED_DEADBAND 80.0f
#define CHASSIS_ID2_M3508_SPEED_DEADBAND 80.0f
#define CHASSIS_ID3_M3508_SPEED_DEADBAND 80.0f
#define CHASSIS_ID4_M3508_SPEED_DEADBAND 80.0f

/* GM6020 转向电机限幅:
 * 角度环输出的是目标转速, 受 CHASSIS_STEERING_ANGLE_MAX_OUT 限制;
 * 速度环输出最终 CAN 电流/电压指令, 受 CHASSIS_STEERING_SPEED_MAX_OUT 限制。
 * 转向“没力”优先增大速度环限幅, 转向“慢”再增大角度环限幅或角度环 Kp。 */
#define CHASSIS_STEERING_ID3_STARTUP_GUARD_ENABLE 1U
#define CHASSIS_STEERING_STARTUP_HOLD_TICKS 50U
#define CHASSIS_STEERING_ANGLE_MAX_OUT 1200.0f
#define CHASSIS_STEERING_SPEED_MAX_OUT 9000.0f
#define CHASSIS_GM6020_ID3_OUTPUT_REVERSE 1U

/* 静止回正防抖:
 * 遥控全零时, 仍允许 IMU 的 yaw 修正让底盘原地回正。
 * 当回正误差进入 1 度内, 认为已经到边界点, 关闭 offset_w 并清 PID 残留;
 * 只有再次被拧出 2 度外才重新启动回正, 形成 1~2 度滞回, 避免边界反复抖动。
 * offset_w 小于 1 时停止对齐, 四个舵轮回到底盘机械对齐角, 不再摆 45 度斜口字。 */
#define CHASSIS_STOP_STEERING_ALIGN_VW_DEADBAND 1.0f
#define CHASSIS_IDLE_YAW_CORRECTION_ENTER_DEADBAND_DEG 1.0f
#define CHASSIS_IDLE_YAW_CORRECTION_EXIT_DEADBAND_DEG 2.0f

/* BMI088 + EKF 姿态解算参数:
 * BMI088 当前配置为加速度 800 Hz、陀螺仪 2000 Hz, 底盘任务按实际 dt 调用 EKF。
 * 初始化时平均若干帧作为静止零偏; yaw 轴零偏单独扣除, pitch/roll 轴零偏放入 EKF bias 状态。
 * yaw 复位不直接改 EKF 四元数, 而是维护 yaw_offset, 这样 ResetYaw 后下一次 EKF 更新不会跳回原角度。 */
#define CHASSIS_IMU_DT_FALLBACK_S 0.001f
#define CHASSIS_IMU_DT_MAX_S 0.05f
#define CHASSIS_IMU_YAW_AXIS_X 0U
#define CHASSIS_IMU_YAW_AXIS_Y 1U
#define CHASSIS_IMU_YAW_AXIS_Z 2U
#define CHASSIS_IMU_YAW_AXIS CHASSIS_IMU_YAW_AXIS_Z
#define CHASSIS_IMU_YAW_SIGN 1.0f
#define CHASSIS_IMU_INIT_SAMPLE_COUNT 512U
#define CHASSIS_IMU_INIT_SAMPLE_DELAY_MS 1U
#define CHASSIS_IMU_EKF_Q_NOISE 10.0f
#define CHASSIS_IMU_EKF_BIAS_NOISE 0.001f
#define CHASSIS_IMU_EKF_ACCEL_NOISE 1000.0f
#define CHASSIS_IMU_EKF_LAMBDA 1.0f
#define CHASSIS_IMU_EKF_ACCEL_LPF 0.01f

/* 底盘行走电机实例（3508 电机）:  */
static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb;

/* 底盘转向电机实例（6020 电机）:  */
static DJIMotor_Instance *motor_steering_lf, *motor_steering_rf, *motor_steering_lb, *motor_steering_rb;

/* 航向锁定 PID 控制器 */
static PID_Instance chassis_follow_pid;

/* 临时目标轮速与角度（用于某些特殊运动模型） */
static float vt_lf, vt_rf, vt_lb, vt_rb;
static float at_lf, at_rf, at_lb, at_rb;

/* 底盘 IMU 内部数据存储 */
static ChassisIMUData_s chassis_imu_data;
static uint8_t chassis_imu_enable_request = 1U;
static AttitudeEKF chassis_imu_ekf;
static float chassis_imu_yaw_offset_deg = 0.0f;
static float chassis_imu_yaw_gyro_bias_rads = 0.0f;

/* 全局底盘控制命令状态 */
ChassisCtrlCmd_s chassis_ctrl_cmd = {
    .imu_enable = 0U,
    .Chassis_IMU_data = &chassis_imu_data,
    .correct_mode = IMU_CORRECT_STRAIGHT,
    .last_yaw = 0.0f,
    .target_yaw = 0.0f,
    .offset_w = 0.0f,
};

/**
 * @brief 将角度规格化至 [-180, 180]
 */
static float ChassisIMU_NormalizeDeg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/**
 * @brief 计算两个角度的最小偏差值
 */
static float ChassisIMU_DiffDeg(float target_deg, float current_deg)
{
    return ChassisIMU_NormalizeDeg(target_deg - current_deg);
}

/* 静止回正进入角度死区时清掉航向 PID:
 * PIDCalculate() 在死区内只清 ITerm/Output, Iout 仍可能保留。
 * 如果不清 Iout, 下一次刚退出死区时会带着旧积分冲一下, 表现为边界点抖动。 */
static void ChassisIMU_ClearCorrectionPID(void)
{
    chassis_follow_pid.Pout = 0.0f;
    chassis_follow_pid.Iout = 0.0f;
    chassis_follow_pid.Dout = 0.0f;
    chassis_follow_pid.ITerm = 0.0f;
    chassis_follow_pid.Output = 0.0f;
    chassis_follow_pid.Last_Output = 0.0f;
    chassis_follow_pid.Last_Dout = 0.0f;
    chassis_follow_pid.Last_ITerm = 0.0f;
}

static float ChassisIMU_SelectYawGyroRaw(const BMI088_Data_t *data)
{
    float yaw_gyro = data->gyro_rads.z;

#if CHASSIS_IMU_YAW_AXIS == CHASSIS_IMU_YAW_AXIS_X
    yaw_gyro = data->gyro_rads.x;
#elif CHASSIS_IMU_YAW_AXIS == CHASSIS_IMU_YAW_AXIS_Y
    yaw_gyro = data->gyro_rads.y;
#else
    yaw_gyro = data->gyro_rads.z;
#endif

    return yaw_gyro;
}

static float ChassisIMU_SelectYawGyro(void)
{
    return (ChassisIMU_SelectYawGyroRaw(&g_bmi088_data) - chassis_imu_yaw_gyro_bias_rads) * CHASSIS_IMU_YAW_SIGN;
}

static float ChassisIMU_GetEKFYawDeg(void)
{
    return ChassisIMU_NormalizeDeg(chassis_imu_ekf.yaw * CHASSIS_IMU_YAW_SIGN + chassis_imu_yaw_offset_deg);
}

/* yaw 角复位策略:
 * EKF 内部 yaw 由陀螺积分得到, 直接改输出变量只能保持一帧。
 * 这里用 yaw_offset 把 EKF yaw 映射到底盘 yaw:
 *   chassis_yaw = ekf_yaw * yaw_sign + yaw_offset
 * ResetYaw(x) 等价于重新计算 yaw_offset, 不破坏 EKF 内部姿态状态。 */
static void ChassisIMU_SetYawWithOffset(float yaw_deg)
{
    yaw_deg = ChassisIMU_NormalizeDeg(yaw_deg);

    if (chassis_imu_ekf.initialized != 0U) {
        chassis_imu_yaw_offset_deg = ChassisIMU_NormalizeDeg(yaw_deg - chassis_imu_ekf.yaw * CHASSIS_IMU_YAW_SIGN);
    } else {
        chassis_imu_yaw_offset_deg = yaw_deg;
    }

    chassis_imu_data.Yaw = yaw_deg;
}

static void ChassisIMU_InitEKF(float ax, float ay, float az, float gyro_bias_x, float gyro_bias_y, float yaw_gyro_bias)
{
    AttitudeEKF_InitFromAccel(&chassis_imu_ekf,
                              ax,
                              ay,
                              az,
                              CHASSIS_IMU_EKF_Q_NOISE,
                              CHASSIS_IMU_EKF_BIAS_NOISE,
                              CHASSIS_IMU_EKF_ACCEL_NOISE,
                              CHASSIS_IMU_EKF_LAMBDA,
                              CHASSIS_IMU_EKF_ACCEL_LPF);

    chassis_imu_ekf.x[4] = gyro_bias_x;
    chassis_imu_ekf.x[5] = gyro_bias_y;
    chassis_imu_ekf.gyro_bias[0] = gyro_bias_x;
    chassis_imu_ekf.gyro_bias[1] = gyro_bias_y;
    chassis_imu_yaw_gyro_bias_rads = yaw_gyro_bias;
}

/* 上电静止校准:
 * 连续读取 BMI088 多帧数据, 平均加速度用于从重力方向初始化 EKF 四元数;
 * 平均 gyro x/y 作为 EKF 可观测 bias 初值, yaw 轴 bias 单独保存后在 gz 上扣除。
 * 注意 yaw 轴没有磁力计/外部航向观测, 只能靠上电静止零偏降低漂移。 */
static BMI088_Status_t ChassisIMU_CalibrateAndInitEKF(void)
{
    BMI088_Status_t status = g_bmi088_status;
    uint16_t valid_count = 0U;
    float sum_ax = 0.0f;
    float sum_ay = 0.0f;
    float sum_az = 0.0f;
    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_yaw_gyro = 0.0f;

    for (uint16_t i = 0U; i < CHASSIS_IMU_INIT_SAMPLE_COUNT; i++) {
        status = BMI088_ReadAll(&g_bmi088_data);
        if (status == BMI088_OK) {
            sum_ax += g_bmi088_data.accel_mps2.x;
            sum_ay += g_bmi088_data.accel_mps2.y;
            sum_az += g_bmi088_data.accel_mps2.z;
            sum_gx += g_bmi088_data.gyro_rads.x;
            sum_gy += g_bmi088_data.gyro_rads.y;
            sum_yaw_gyro += ChassisIMU_SelectYawGyroRaw(&g_bmi088_data);
            valid_count++;
        }

        if (CHASSIS_IMU_INIT_SAMPLE_DELAY_MS > 0U) {
            HAL_Delay(CHASSIS_IMU_INIT_SAMPLE_DELAY_MS);
        }
    }

    if (valid_count == 0U) {
        return status;
    }

    const float inv_count = 1.0f / (float)valid_count;
    ChassisIMU_InitEKF(sum_ax * inv_count,
                       sum_ay * inv_count,
                       sum_az * inv_count,
                       sum_gx * inv_count,
                       sum_gy * inv_count,
                       sum_yaw_gyro * inv_count);

    return BMI088_OK;
}

/**
 * @brief 底盘 IMU 数据及其控制结构体初始化
 */
static void ChassisIMU_Init(void)
{
    chassis_imu_data.Yaw = 0.0f;
    chassis_imu_data.Pitch = 0.0f;
    chassis_imu_data.Roll = 0.0f;
    chassis_imu_data.GyroZ = 0.0f;
    chassis_imu_data.status = ChassisIMU_CalibrateAndInitEKF();
    g_bmi088_status = chassis_imu_data.status;
    chassis_imu_data.online = (chassis_imu_data.status == BMI088_OK) ? 1U : 0U;
    ChassisIMU_SetYawWithOffset(0.0f);

    chassis_ctrl_cmd.Chassis_IMU_data = &chassis_imu_data;
    chassis_ctrl_cmd.imu_enable = (chassis_imu_enable_request != 0U && chassis_imu_data.online != 0U) ? 1U : 0U;
    chassis_ctrl_cmd.correct_mode = IMU_CORRECT_STRAIGHT;
    chassis_ctrl_cmd.last_yaw = chassis_imu_data.Yaw;
    chassis_ctrl_cmd.target_yaw = chassis_imu_data.Yaw;
    chassis_ctrl_cmd.offset_w = 0.0f;
}

/**
 * @brief 更新底盘 IMU 姿态信息，通过 BMI088 数据驱动 EKF 姿态解算
 * @param dt_s 采样周期 (秒)
 */
/* BMI088 姿态更新:
 * 每个底盘周期读取 BMI088, 用实际 dt 驱动 AttitudeEKF_Update().
 * gz 先扣除上电静止平均得到的 yaw 零偏, 让无磁力计的 yaw 积分漂移尽量小。
 * 车体坐标与 EKF 输出坐标实测 pitch/roll 互换, 所以对外输出:
 *   Pitch = ekf.roll
 *   Roll  = ekf.pitch
 * 这里不改 EKF 内部四元数, 只做底盘数据层适配。 */
static void ChassisIMU_Update(float dt_s)
{
    BMI088_Status_t status;
    uint8_t was_online;

    if (dt_s <= 0.0f) {
        return;
    }

    was_online = chassis_imu_data.online;
    status = BMI088_ReadAll(&g_bmi088_data);
    g_bmi088_status = status;
    chassis_imu_data.status = status;

    if (status != BMI088_OK) {
        chassis_imu_data.online = 0U;
        chassis_ctrl_cmd.imu_enable = 0U;
        chassis_ctrl_cmd.offset_w = 0.0f;
        return;
    }

    chassis_imu_data.online = 1U;
    chassis_imu_data.GyroZ = ChassisIMU_SelectYawGyro();

    if (chassis_imu_ekf.initialized == 0U) {
        ChassisIMU_InitEKF(g_bmi088_data.accel_mps2.x,
                           g_bmi088_data.accel_mps2.y,
                           g_bmi088_data.accel_mps2.z,
                           g_bmi088_data.gyro_rads.x,
                           g_bmi088_data.gyro_rads.y,
                           ChassisIMU_SelectYawGyroRaw(&g_bmi088_data));
        ChassisIMU_SetYawWithOffset(chassis_imu_data.Yaw);
    }

    AttitudeEKF_Update(&chassis_imu_ekf,
                       g_bmi088_data.gyro_rads.x,
                       g_bmi088_data.gyro_rads.y,
                       g_bmi088_data.gyro_rads.z - chassis_imu_yaw_gyro_bias_rads,
                       g_bmi088_data.accel_mps2.x,
                       g_bmi088_data.accel_mps2.y,
                       g_bmi088_data.accel_mps2.z,
                       dt_s);

    chassis_imu_data.Yaw = ChassisIMU_GetEKFYawDeg();
    chassis_imu_data.Pitch = chassis_imu_ekf.roll;
    chassis_imu_data.Roll = chassis_imu_ekf.pitch;
    chassis_ctrl_cmd.imu_enable = (chassis_imu_enable_request != 0U) ? 1U : 0U;

    if (was_online == 0U) {
        chassis_ctrl_cmd.last_yaw = chassis_imu_data.Yaw;
        chassis_ctrl_cmd.target_yaw = chassis_imu_data.Yaw;
        chassis_ctrl_cmd.offset_w = 0.0f;
    }
}

void ChassisIMU_Enable(uint8_t enable)
{
    chassis_imu_enable_request = (enable != 0U) ? 1U : 0U;
    chassis_ctrl_cmd.imu_enable = (chassis_imu_enable_request != 0U && chassis_imu_data.online != 0U) ? 1U : 0U;
}

void ChassisIMU_SetCorrectMode(ChassisIMUCorrectMode_e mode)
{
    chassis_ctrl_cmd.correct_mode = mode;
    chassis_ctrl_cmd.last_yaw = chassis_imu_data.Yaw;
    chassis_ctrl_cmd.target_yaw = chassis_imu_data.Yaw;
    chassis_ctrl_cmd.offset_w = 0.0f;
}

void ChassisIMU_ResetYaw(float yaw_deg)
{
    ChassisIMU_SetYawWithOffset(yaw_deg);
    chassis_ctrl_cmd.last_yaw = chassis_imu_data.Yaw;
    chassis_ctrl_cmd.target_yaw = chassis_imu_data.Yaw;
    chassis_ctrl_cmd.offset_w = 0.0f;
}

static float ChassisSteeringAngle(DJIMotor_Instance *motor)
{
    if (motor == NULL) {
        return 0.0f;
    }

    return motor->measure.total_angle;
}

static uint8_t ChassisSteeringId3StartupGuard(void)
{
#if CHASSIS_STEERING_ID3_STARTUP_GUARD_ENABLE
    static uint8_t hold_ticks = 0U;

    if (motor_steering_lf == NULL || motor_steering_lf->feedback_initialized == 0U) {
        hold_ticks = 0U;
        if (motor_steering_lf != NULL) {
            DJIMotorStop(motor_steering_lf);
        }
        return 0U;
    }

    if (hold_ticks < CHASSIS_STEERING_STARTUP_HOLD_TICKS) {
        hold_ticks++;
        DJIMotorSetRef(motor_steering_lf, motor_steering_lf->measure.total_angle);
        DJIMotorEnable(motor_steering_lf);
        return 0U;
    }

    return 1U;
#else
    return 1U;
#endif
}

/* 行走轮速度下发:
 * vt_* 来自舵轮运动学, 单位与 M3508 speed feedback 保持一致。
 * 小于 deadband 时给 0 速并停止该行走轮, 避免极小速度下 PID 拉扯;
 * deadband 不宜过大, 否则低速移动会在停/启之间切换而抖。 */
static void ChassisSetDriveMotorRef(DJIMotor_Instance *motor, float speed, float deadband)
{
    if (motor == NULL) {
        return;
    }

    if (fabsf(speed) < deadband) {
        DJIMotorSetRef(motor, 0.0f);
        DJIMotorStop(motor);
    } else {
        DJIMotorEnable(motor);
        DJIMotorSetRef(motor, speed);
    }
}

void ChassisInit()
{
    // 四个行走电机参数基本一致，主要区别是 CAN ID 和电机方向。
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.fdcan_handle   = &hfdcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1.7, // 3
                .Ki            = 0.27, // 0.5
                .Kd            = 0.005,   // 0
                .IntegralLimit = 3000,//5000
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 10000,
            },
            .current_PID = {
                .Kp            = 1, // 1
                .Ki            = 0.01,   // 0
                .Kd            = 0,
                .IntegralLimit = 3000,//3000
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = CURRENT_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508,
    };

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis_motor_config.controller_param_init_config.speed_PID.DeadBand   = CHASSIS_ID3_M3508_SPEED_DEADBAND;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis_motor_config.controller_param_init_config.speed_PID.DeadBand   = CHASSIS_ID4_M3508_SPEED_DEADBAND;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 1;
	       chassis_motor_config.controller_param_init_config.speed_PID.Kp         =1.1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis_motor_config.controller_param_init_config.speed_PID.DeadBand   = CHASSIS_ID1_M3508_SPEED_DEADBAND;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
            chassis_motor_config.controller_param_init_config.speed_PID.Kp         =2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis_motor_config.controller_param_init_config.speed_PID.DeadBand   = CHASSIS_ID2_M3508_SPEED_DEADBAND;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);


    // 6020 转向电机初始化。
    Motor_Init_Config_s chassis_motor_steering_config = {
        .can_init_config.fdcan_handle   = &hfdcan2,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 13,
                .Ki                = 0.34,
                .Kd                = 0,
                .CoefA             = 5,
                .CoefB             = 0.1,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .IntegralLimit     = 1000,
                .MaxOut            = CHASSIS_STEERING_ANGLE_MAX_OUT,
                .Derivative_LPF_RC = 0.001,
                .DeadBand          = 1,
            },
            .speed_PID = {
                .Kp            = 50,
                .Ki            = 3,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
                .IntegralLimit = 4000,
                .MaxOut        = CHASSIS_STEERING_SPEED_MAX_OUT,
                .Output_LPF_RC = 0.03,
            },

        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .angle_mode            = MOTOR_ANGLE_MODE_SINGLE_TURN,
        },
        .motor_type = GM6020,
    };
    chassis_motor_steering_config.can_init_config.tx_id = 3;
#if CHASSIS_GM6020_ID3_OUTPUT_REVERSE
    chassis_motor_steering_config.controller_setting_init_config.output_reverse_flag = MOTOR_DIRECTION_REVERSE;
#endif
    motor_steering_lf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.controller_setting_init_config.output_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_steering_config.can_init_config.tx_id = 4;
    motor_steering_rf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 1;
    motor_steering_lb                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 2;
    motor_steering_rb                                   = DJIMotorInit(&chassis_motor_steering_config);
#if CHASSIS_STEERING_ID3_STARTUP_GUARD_ENABLE
    if (motor_steering_lf != NULL) {
        DJIMotorStop(motor_steering_lf);
    }
#endif

            PID_Init_Config_s chassis_follow_pid_conf = {
        .Kp                = 150, // 6
        .Ki                = 0.1f,
        .Kd                = 17, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ErrorHandle,
        .IntegralLimit     = 500, // 200
        .MaxOut            = 25000,
        .Derivative_LPF_RC = 0.01, // 0.01
    };

    PIDInit(&chassis_follow_pid, &chassis_follow_pid_conf);
    ChassisIMU_Init();

}

/**
 * @brief 选择最小转角，并在必要时反转轮速，避免转向电机绕远路。
 *        例如上次角度为 0°、目标角度为 135° 时，优先选择反向行驶并转到 -45°。
 * @param angle 目标角度指针。
 * @param last_angle 上一次实际角度指针。
 * @param speed 轮速指针；当转向角翻转 180° 时同步反向。
 */
static void MinmizeRotation(float *angle, const float *last_angle, float *speed)
{
    float rotation = ChassisIMU_NormalizeDeg(*angle - *last_angle);

    if (rotation > 90) {
        *angle -= 180;
        *speed = -(*speed);
    } else if (rotation < -90) {
        *angle += 180;
        *speed = -(*speed);
    }
}

/**
 * @brief 统一的 IMU 航向修正计算，支持直行保持、旋转跟踪和混合模式。
 * @param target_vw 目标角速度指令。
 * @return 叠加到底盘角速度上的修正量 offset_w。
 */
/* IMU 航向修正:
 * 当前只用 yaw 做底盘姿态矫正, pitch/roll 仅输出给上层观察。
 * STRAIGHT: 没有主动旋转指令时, 锁住 last_yaw, 输出 offset_w 抵消外力扭转;
 * ROTATION: 有旋转指令时积分 target_yaw, 用 yaw 闭环跟随目标角;
 * HYBRID: 旋转时保留一半航向修正, 直行时完整锁航向。
 * 返回值 offset_w 会叠加到底盘角速度 vw 上。 */
static float UpdateIMUCorrection(float target_vw)
{
    if(!chassis_ctrl_cmd.imu_enable) {
        return 0;
    }

    float current_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;
    float offset = 0;

    switch(chassis_ctrl_cmd.correct_mode)
    {
        case IMU_CORRECT_STRAIGHT:
            if(fabsf(target_vw) < 100.0f) {
                float yaw_error = ChassisIMU_DiffDeg(chassis_ctrl_cmd.last_yaw, current_yaw);
                offset = PIDCalculate(&chassis_follow_pid, 0.0f, yaw_error);
            } else {
                chassis_ctrl_cmd.last_yaw = current_yaw;
                offset = 0;
            }
            break;

        case IMU_CORRECT_ROTATION:
            if(fabsf(target_vw) > 100.0f) {
                chassis_ctrl_cmd.target_yaw = ChassisIMU_NormalizeDeg(chassis_ctrl_cmd.target_yaw + target_vw * 0.001f);
            }
            float target_error = ChassisIMU_DiffDeg(chassis_ctrl_cmd.target_yaw, current_yaw);
            offset = PIDCalculate(&chassis_follow_pid, 0.0f, target_error);
            break;

        case IMU_CORRECT_HYBRID:
        {
            float yaw_error = ChassisIMU_DiffDeg(chassis_ctrl_cmd.last_yaw, current_yaw);

            if(fabsf(target_vw) < 100.0f) {
                offset = PIDCalculate(&chassis_follow_pid, yaw_error, 0);
            } else {
                offset = PIDCalculate(&chassis_follow_pid, yaw_error, 0) * 0.5f;
                chassis_ctrl_cmd.last_yaw = current_yaw;
            }
            break;
        }

        default:
            offset = 0;
            break;
    }

    return offset;
}

/* 静止回正误差来源:
 * 直行/混合模式以 last_yaw 为锁定角;
 * 旋转模式以 target_yaw 为目标角。
 * 静止死区用这个误差判断是否继续原地回正。 */
static float ChassisIMU_GetCorrectionYawError(void)
{
    float current_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;

    if (chassis_ctrl_cmd.correct_mode == IMU_CORRECT_ROTATION) {
        return ChassisIMU_DiffDeg(chassis_ctrl_cmd.target_yaw, current_yaw);
    }

    return ChassisIMU_DiffDeg(chassis_ctrl_cmd.last_yaw, current_yaw);
}

/**
 * @brief 舵轮底盘运动学解算。
 * @param vx 前后方向线速度指令。
 * @param vy 左右方向线速度指令。
 * @param vw 角速度指令。
 * @note 当前使用4轮舵轮模式：3 号轮为左前轮，1 号轮为左后轮，2 号轮为右后轮，4 号轮为右前轮。
 */
/* 四舵轮运动学与静止回正:
 * 输入 vx/vy/vw 先叠加 IMU 航向修正 offset_w, 再解算每个舵轮速度 vt_* 和角度 at_*。
 * 遥控全零但 yaw 被外力拧偏时, 不进入停车分支, 允许 offset_w 让底盘原地回正。
 * 回正误差进入 1 度内后锁住 idle_yaw_correction_hold, 清 PID 并停止 offset_w;
 * 只有偏出 2 度外才重新启动回正, 避免在目标边界附近来回抖。
 * 停止时四个舵轮回到底盘对齐角, 不再摆 45 度斜口字。 */
void SteeringWheelKinematics(float vx, float vy, float vw)
{
    float chassis_vx = vx;
    float chassis_vy = vy;
    float chassis_vw = vw;
    static uint8_t first_run_kinematics = 1;
    static uint8_t idle_yaw_correction_hold = 0U;
    float offset_lf = 0.0f, offset_rf = 0.0f, offset_lb = 0.0f, offset_rb = 0.0f;
    float at_lf_last = 0.0f, at_rf_last = 0.0f, at_lb_last = 0.0f, at_rb_last = 0.0f;
    uint8_t id3_ready = 0U;
    uint8_t manual_idle = ((vx == 0.0f) && (vy == 0.0f) && (vw == 0.0f)) ? 1U : 0U;
    uint8_t stop_align_ready = 0U;
    float idle_yaw_error = 0.0f;

    id3_ready = ChassisSteeringId3StartupGuard();

    at_lf_last = ChassisSteeringAngle(motor_steering_lf);
    at_rf_last = ChassisSteeringAngle(motor_steering_rf);
    at_lb_last = ChassisSteeringAngle(motor_steering_lb);
    at_rb_last = ChassisSteeringAngle(motor_steering_rb);

    if (first_run_kinematics) {
        chassis_ctrl_cmd.last_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;
        first_run_kinematics = 0;
    }

    if (manual_idle != 0U) {
        idle_yaw_error = fabsf(ChassisIMU_GetCorrectionYawError());

        if (idle_yaw_correction_hold != 0U) {
            if (idle_yaw_error > CHASSIS_IDLE_YAW_CORRECTION_EXIT_DEADBAND_DEG) {
                idle_yaw_correction_hold = 0U;
            }
        } else if (idle_yaw_error < CHASSIS_IDLE_YAW_CORRECTION_ENTER_DEADBAND_DEG) {
            idle_yaw_correction_hold = 1U;
            ChassisIMU_ClearCorrectionPID();
        }
    } else {
        idle_yaw_correction_hold = 0U;
    }

    if (idle_yaw_correction_hold != 0U) {
        chassis_ctrl_cmd.offset_w = 0.0f;
        chassis_vw = 0.0f;
    } else {
        chassis_ctrl_cmd.offset_w = UpdateIMUCorrection(vw);
        chassis_vw = vw + chassis_ctrl_cmd.offset_w;
    }

    stop_align_ready = (manual_idle != 0U && fabsf(chassis_vw) < CHASSIS_STOP_STEERING_ALIGN_VW_DEADBAND) ? 1U : 0U;
    if (stop_align_ready != 0U) {
        chassis_ctrl_cmd.offset_w = 0.0f;
        chassis_vw = 0.0f;
    }

    float w = chassis_vw;
    float temp_x = chassis_vx - w;
    float temp_y = chassis_vy - w;

    vt_lf = sqrtf(temp_x * temp_x + temp_y * temp_y);
    offset_lf = atan2f(temp_y, temp_x) * RAD_2_DEGREE;

    temp_y = chassis_vy + w;
    vt_lb = sqrtf(temp_x * temp_x + temp_y * temp_y);
    offset_lb = atan2f(temp_y, temp_x) * RAD_2_DEGREE;

    temp_x = chassis_vx + w;
    vt_rb = sqrtf(temp_x * temp_x + temp_y * temp_y);
    offset_rb = atan2f(temp_y, temp_x) * RAD_2_DEGREE;

    temp_y = chassis_vy - w;
    vt_rf = sqrtf(temp_x * temp_x + temp_y * temp_y);
    offset_rf = atan2f(temp_y, temp_x) * RAD_2_DEGREE;

    at_lf = STEERING_CHASSIS_ALIGN_ANGLE_LF + offset_lf;
    at_rf = STEERING_CHASSIS_ALIGN_ANGLE_RF + offset_rf;
    at_lb = STEERING_CHASSIS_ALIGN_ANGLE_LB + offset_lb;
    at_rb = STEERING_CHASSIS_ALIGN_ANGLE_RB + offset_rb;

    ANGLE_LIMIT_360_TO_180_ABS(at_lf);
    ANGLE_LIMIT_360_TO_180_ABS(at_rf);
    ANGLE_LIMIT_360_TO_180_ABS(at_lb);
    ANGLE_LIMIT_360_TO_180_ABS(at_rb);

    if (stop_align_ready == 0U) {
        MinmizeRotation(&at_lf, &at_lf_last, &vt_lf);
        MinmizeRotation(&at_rf, &at_rf_last, &vt_rf);
        MinmizeRotation(&at_lb, &at_lb_last, &vt_lb);
        MinmizeRotation(&at_rb, &at_rb_last, &vt_rb);
    }

    if (id3_ready) {
        DJIMotorEnable(motor_steering_lf);
        DJIMotorSetRef(motor_steering_lf, at_lf);
    } else {
        vt_lf = 0.0f;
    }
    DJIMotorSetRef(motor_steering_rf, at_rf);
    DJIMotorSetRef(motor_steering_lb, at_lb);
    DJIMotorSetRef(motor_steering_rb, at_rb);

    ChassisSetDriveMotorRef(motor_lf, vt_lf, CHASSIS_ID3_M3508_SPEED_DEADBAND);
    ChassisSetDriveMotorRef(motor_rf, vt_rf, CHASSIS_ID4_M3508_SPEED_DEADBAND);
    ChassisSetDriveMotorRef(motor_lb, vt_lb, CHASSIS_ID1_M3508_SPEED_DEADBAND);
    ChassisSetDriveMotorRef(motor_rb, vt_rb, CHASSIS_ID2_M3508_SPEED_DEADBAND);
}
void ChassisTask(void)
{
    float vx = 0.0f, vy = 0.0f, vw = 0.0f;
    static uint32_t imu_update_cnt = 0U;
    float imu_dt_s = DWT_GetDeltaT(&imu_update_cnt);

    if ((imu_dt_s <= 0.0f) || (imu_dt_s > CHASSIS_IMU_DT_MAX_S)) {
        imu_dt_s = CHASSIS_IMU_DT_FALLBACK_S;
    }
    ChassisIMU_Update(imu_dt_s);

    if (remote_data != NULL) {
        // Left stick X is mounted as the forward/back channel on this remote.
        // Pushing forward makes rocker_l_ negative, so map it to positive vx.
        vx = -(float)remote_data->rocker_l_ / REMOTE_STICK_RANGE * REMOTE_MAX_LINEAR;
        // Left stick Y is used as the lateral channel.
        vy = (float)remote_data->rocker_l1 / REMOTE_STICK_RANGE * REMOTE_MAX_LINEAR;
        // 右摇杆 X轴 → 旋转角速度 vw
        vw = 0;

        // 死区
        if (fabsf(vx) < REMOTE_DEADBAND) vx = 0.0f;
        if (fabsf(vy) < REMOTE_DEADBAND) vy = 0.0f;
        if (fabsf(vw) < REMOTE_DEADBAND) vw = 0.0f;
    }

    SteeringWheelKinematics(vx, vy, vw);
		
		
//	DJIMotorSetRef(motor_steering_rf,STEERING_CHASSIS_ALIGN_ANGLE_1);
//	DJIMotorSetRef(motor_steering_lf,STEERING_CHASSIS_ALIGN_ANGLE_3);
//	DJIMotorSetRef(motor_steering_rb,STEERING_CHASSIS_ALIGN_ANGLE_2);
}
