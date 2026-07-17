/**
 * @file chassis.c
 * @author 
 * @brief 底盘控制实现，包含电机初始化、IMU 处理和运动学分解，基本完善
 * @version 0.1
 * @date 2024-xx-xx
 */

#include "chassis.h"
#include "DJI_motor.h"
#include "main.h"
#include "attitude_ekf.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "remote_logic_profile.h"
#include "robot_def.h"
#include <math.h>

#define CHASSIS_ID1_M3508_SPEED_DEADBAND 80.0f        // ID1 行走 3508 速度死区
#define CHASSIS_ID2_M3508_SPEED_DEADBAND 80.0f        // ID2 行走 3508 速度死区
#define CHASSIS_ID3_M3508_SPEED_DEADBAND 80.0f        // ID3 行走 3508 速度死区
#define CHASSIS_ID4_M3508_SPEED_DEADBAND 80.0f        // ID4 行走 3508 速度死区
#define CHASSIS_STEERING_ANGLE_MAX_OUT 20000.0f        // 舵向角度环最大输出
#define CHASSIS_STEERING_SPEED_MAX_OUT 20000.0f       // 舵向正常速度环最大输出
#define CHASSIS_STEERING_CURRENT_MAX_OUT 20000.0f     // 舵向电流环最大输出
#define CHASSIS_STEERING_HOME_SPEED_REF 3500.0f       // 舵向归零时的速度给定
#define CHASSIS_STEERING_ANGLE_KP_LF 5.5f             // 左前舵向 ID7 角度环 Kp
#define CHASSIS_STEERING_ANGLE_KI_LF 1.5f             // 左前舵向 ID7 角度环 Ki
#define CHASSIS_STEERING_ANGLE_KD_LF 0.2f             // 左前舵向 ID7 角度环 Kd
#define CHASSIS_STEERING_ANGLE_INTEGRAL_LIMIT_LF 20000.0f // 左前舵向 ID7 角度环积分限幅
#define CHASSIS_STEERING_ANGLE_KP_RF 6.2f             // 右前舵向 ID8 角度环 Kp
#define CHASSIS_STEERING_ANGLE_KI_RF 2.2f             // 右前舵向 ID8 角度环 Ki
#define CHASSIS_STEERING_ANGLE_KD_RF 0.1f             // 右前舵向 ID8 角度环 Kd
#define CHASSIS_STEERING_ANGLE_INTEGRAL_LIMIT_RF 20000.0f // 右前舵向 ID8 角度环积分限幅
#define CHASSIS_STEERING_ANGLE_KP_LB 5.5f             // 左后舵向 ID5 角度环 Kp
#define CHASSIS_STEERING_ANGLE_KI_LB 0.05f             // 左后舵向 ID5 角度环 Ki
#define CHASSIS_STEERING_ANGLE_KD_LB 0.15f             // 左后舵向 ID5 角度环 Kd
#define CHASSIS_STEERING_ANGLE_INTEGRAL_LIMIT_LB 20000.0f // 左后舵向 ID5 角度环积分限幅
#define CHASSIS_STEERING_ANGLE_KP_RB 6.5f             // 右后舵向 ID6 角度环 Kp
#define CHASSIS_STEERING_ANGLE_KI_RB 2.5f             // 右后舵向 ID6 角度环 Ki
#define CHASSIS_STEERING_ANGLE_KD_RB 0.1f             // 右后舵向 ID6 角度环 Kd
#define CHASSIS_STEERING_ANGLE_INTEGRAL_LIMIT_RB 20000.0f // 右后舵向 ID6 角度环积分限幅
#define CHASSIS_STEERING_ANGLE_COEF_A 5.0f            // 舵向角度环变积分参数 A
#define CHASSIS_STEERING_ANGLE_COEF_B 0.1f            // 舵向角度环变积分参数 B
#define CHASSIS_STEERING_ANGLE_DERIVATIVE_LPF_RC 0.001f // 舵向角度环微分低通滤波 RC
#define CHASSIS_STEERING_ANGLE_DEADBAND 1.0f          // 舵向角度环死区
#define CHASSIS_STEERING_SPEED_KP_LF 5.5f             // 左前舵向 ID7 正常速度环 Kp
#define CHASSIS_STEERING_SPEED_KI_LF 0.001f           // 左前舵向 ID7 正常速度环 Ki
#define CHASSIS_STEERING_SPEED_KD_LF 0.001f           // 左前舵向 ID7 正常速度环 Kd
#define CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_LF 100.0f // 左前舵向 ID7 正常速度环积分限幅
#define CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_LF 0.08f // 左前舵向 ID7 正常速度环输出低通滤波 RC
#define CHASSIS_STEERING_SPEED_KP_RF 5.5f             // 右前舵向 ID8 正常速度环 Kp
#define CHASSIS_STEERING_SPEED_KI_RF 0.001f           // 右前舵向 ID8 正常速度环 Ki
#define CHASSIS_STEERING_SPEED_KD_RF 0.001f           // 右前舵向 ID8 正常速度环 Kd
#define CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_RF 100.0f // 右前舵向 ID8 正常速度环积分限幅
#define CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_RF 0.08f // 右前舵向 ID8 正常速度环输出低通滤波 RC
#define CHASSIS_STEERING_SPEED_KP_LB 5.5f             // 左后舵向 ID5 正常速度环 Kp
#define CHASSIS_STEERING_SPEED_KI_LB 0.001f           // 左后舵向 ID5 正常速度环 Ki
#define CHASSIS_STEERING_SPEED_KD_LB 0.001f           // 左后舵向 ID5 正常速度环 Kd
#define CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_LB 100.0f // 左后舵向 ID5 正常速度环积分限幅
#define CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_LB 0.08f // 左后舵向 ID5 正常速度环输出低通滤波 RC
#define CHASSIS_STEERING_SPEED_KP_RB 6.5f             // 右后舵向 ID6 正常速度环 Kp
#define CHASSIS_STEERING_SPEED_KI_RB 0.01f           // 右后舵向 ID6 正常速度环 Ki
#define CHASSIS_STEERING_SPEED_KD_RB 0.001f           // 右后舵向 ID6 正常速度环 Kd
#define CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_RB 100.0f // 右后舵向 ID6 正常速度环积分限幅
#define CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_RB 0.08f // 右后舵向 ID6 正常速度环输出低通滤波 RC
#define CHASSIS_STEERING_CURRENT_KP 1.0f              // 舵向电流环 Kp，四个舵电机公用
#define CHASSIS_STEERING_CURRENT_KI 0.01f             // 舵向电流环 Ki，四个舵电机公用
#define CHASSIS_STEERING_CURRENT_KD 0.0f              // 舵向电流环 Kd，四个舵电机公用
#define CHASSIS_STEERING_CURRENT_INTEGRAL_LIMIT 50000.0f // 舵向电流环积分限幅，四个舵电机公用
#define CHASSIS_STEERING_HOME_SPEED_KP 3.5f           // 舵向归零速度环 Kp，四个舵电机公用
#define CHASSIS_STEERING_HOME_SPEED_KI 0.0f           // 舵向归零速度环 Ki，四个舵电机公用
#define CHASSIS_STEERING_HOME_SPEED_KD 0.0f           // 舵向归零速度环 Kd，四个舵电机公用
#define CHASSIS_STEERING_HOME_SPEED_MAX_OUT 15000.0f  // 舵向归零速度环最大输出，四个舵电机公用
#define CHASSIS_STEERING_HOME_SPEED_LPF_RC 0.08f      // 舵向归零速度环输出低通滤波 RC，四个舵电机公用
#define CHASSIS_STEERING_HOME_UNBLOCK_CONFIRM_MS 5U  // 光电门离开遮挡后的确认时间
#define CHASSIS_STEERING_HOME_CONFIRM_MS 5U           // 光电门再次遮挡后的确认时间
#define CHASSIS_STEERING_HOME_SETTLE_MS 1000U         // 归零后停稳等待时间
#define CHASSIS_STEERING_HOME_TIMEOUT_MS 15000U       // 单个舵轮归零超时时间
#define CHASSIS_STEERING_ALIGN_ENABLE 1U              // 舵轮归零后是否转到目标 ECD: 1=正常闭环, 0=停在归零点
#define CHASSIS_STEERING_PHOTO_GATE_BLOCKED GPIO_PIN_RESET // 光电门遮挡电平
#if REMOTE_LOGIC_PROFILE == REMOTE_LOGIC_PROFILE_ALT
#define CHASSIS_IMU_CORRECTION_ENABLE 0U              // ALT 遥控逻辑关闭 IMU 航向修正
#else
#define CHASSIS_IMU_CORRECTION_ENABLE 0U              // CURRENT 遥控逻辑暂时关闭 IMU 航向修正
#endif
#define CHASSIS_STOP_STEERING_ALIGN_VW_DEADBAND 0.2f  // 停车对正时角速度死区
#define CHASSIS_IDLE_YAW_CORRECTION_HOLD_ENABLE 0U    // 静止且航向误差很小时是否暂停 IMU 闭环: 1=暂停, 0=全程闭环
#define CHASSIS_IDLE_YAW_CORRECTION_ENTER_DEADBAND_DEG 1.0f // 静止航向保持进入死区
#define CHASSIS_IDLE_YAW_CORRECTION_EXIT_DEADBAND_DEG 2.0f  // 静止航向保持退出死区
#define CHASSIS_IMU_DT_FALLBACK_S 0.001f              // IMU 更新周期异常时的备用周期
#define CHASSIS_IMU_DT_MAX_S 0.05f                    // IMU 更新周期最大允许值
#define CHASSIS_IMU_YAW_AXIS_X 0U                     // IMU 偏航轴选择 X
#define CHASSIS_IMU_YAW_AXIS_Y 1U                     // IMU 偏航轴选择 Y
#define CHASSIS_IMU_YAW_AXIS_Z 2U                     // IMU 偏航轴选择 Z
#define CHASSIS_IMU_YAW_AXIS CHASSIS_IMU_YAW_AXIS_Z   // 当前 IMU 偏航轴
#define CHASSIS_IMU_YAW_SIGN 1.0f                     // IMU 偏航方向符号
#define CHASSIS_IMU_CORRECTION_SIGN (-1.0f)           // IMU 闭环修正输出方向
#define CHASSIS_IMU_INIT_SAMPLE_COUNT 512U            // IMU 初始化采样次数
#define CHASSIS_IMU_INIT_SAMPLE_DELAY_MS 1U           // IMU 初始化采样间隔
#define CHASSIS_IMU_EKF_Q_NOISE 10.0f                 // IMU EKF 过程噪声
#define CHASSIS_IMU_EKF_BIAS_NOISE 0.001f             // IMU EKF 零偏噪声
#define CHASSIS_IMU_EKF_ACCEL_NOISE 1000.0f           // IMU EKF 加速度噪声
#define CHASSIS_IMU_EKF_LAMBDA 1.0f                   // IMU EKF Lambda 参数
#define CHASSIS_IMU_EKF_ACCEL_LPF 0.01f               // IMU EKF 加速度低通系数

/* 底盘行走电机实例（3508 电机）:  */
static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb;

/* 底盘转向电机实例（3508 电机）:  */
static DJIMotor_Instance *motor_steering_lf, *motor_steering_rf, *motor_steering_lb, *motor_steering_rb;

typedef enum {
    CHASSIS_STEERING_LF = 0,
    CHASSIS_STEERING_RF,
    CHASSIS_STEERING_LB,
    CHASSIS_STEERING_RB,
    CHASSIS_STEERING_COUNT,
} ChassisSteeringIndex_e;

#define CHASSIS_SINGLE_SWERVE_TEST_ENABLE 0U          // 单舵轮测试开关: 1=只测试右前舵轮, 0=四舵轮全部参与
#define CHASSIS_SINGLE_SWERVE_TEST_INDEX CHASSIS_STEERING_RF // 单舵轮测试对象: 右前舵轮, 行走 ID1 + 舵向 ID8
#define CHASSIS_LB_SWERVE_TEST_ENABLE 1U              // 左后舵轮测试开关: 1=参与测试, 0=左后舵向/行走都保持停止
#define CHASSIS_DRIVE_MOTOR_TEST_ENABLE 1U            // 行走电机测试开关: 1=行走电机按解算输出, 0=四个行走电机保持停止
#define CHASSIS_REMOTE_TIMEOUT_MS 300U                // 遥控器超时时间: 超过该时间未收到新帧则底盘参考值清零
#define CHASSIS_REMOTE_VX_SIGN (-1.0f)                // 前后方向修正: 右摇杆上推应前进
#define CHASSIS_REMOTE_VY_SIGN (1.0f)                 // 左右平移方向修正
#define CHASSIS_REMOTE_VW_SIGN (1.0f)                 // 旋转方向修正: 左摇杆X控制角速度
#define CHASSIS_REMOTE_LINEAR_DEADBAND 25.0f          // 前后/平移摇杆原始值死区
#define CHASSIS_REMOTE_ANGULAR_DEADBAND 50.0f         // 旋转摇杆原始值死区
#define CHASSIS_FINE_CONTROL_ENABLE 0U                // SB 精调模式暂时禁用: SB 现在用于 catch/arm 区域切换
#define CHASSIS_FINE_CONTROL_SB_POS 2U                // SB 拨到该档位时启用底盘精调模式
#define CHASSIS_FINE_LINEAR_DEADBAND 6.0f             // 精调模式前后/平移摇杆原始值死区
#define CHASSIS_FINE_ANGULAR_DEADBAND 10.0f           // 精调模式旋转摇杆原始值死区
#define CHASSIS_FINE_LINEAR_SCALE 0.25f               // 精调模式线速度比例
#define CHASSIS_FINE_ANGULAR_SCALE 0.25f              // 精调模式角速度比例
#define CHASSIS_FINE_TRANSLATION_DRIVE_START 10.0f    // 精调模式平移摇杆启动行走轮输出阈值
#define CHASSIS_ROTATE_RELEASE_SETTLE_ENABLE 1U       // 快速旋转松手后短暂跟随当前航向,避免 IMU 立刻反拉
#define CHASSIS_ROTATE_RELEASE_SETTLE_MS 180U         // 旋转释放后的航向重捕获时间
#define CHASSIS_MANUAL_ROTATE_ENTER_VW 100.0f         // 进入手动旋转状态的角速度阈值
#define CHASSIS_MANUAL_ROTATE_EXIT_VW 60.0f           // 退出手动旋转状态的角速度阈值
#define CHASSIS_REMOTE_TRANSLATION_DRIVE_START 80.0f  // 平移摇杆超过该原始值后才给行走轮输出
#if REMOTE_LOGIC_PROFILE == REMOTE_LOGIC_PROFILE_ALT
#define CHASSIS_TRANSLATION_DIR_CHANGE_ENABLE 0U      // ALT 遥控逻辑关闭平移方向突变限速
#else
#define CHASSIS_TRANSLATION_DIR_CHANGE_ENABLE 1U      // CURRENT 遥控逻辑启用平移方向突变限速
#endif
#define CHASSIS_TRANSLATION_DIR_CHANGE_TRIGGER_DEG 50.0f // 触发平移方向突变限速的方向变化角
#define CHASSIS_TRANSLATION_DIR_CHANGE_TIME_MS 160U   // 平移方向突变限速恢复时间
#define CHASSIS_TRANSLATION_DIR_CHANGE_MIN_SCALE 0.20f // 平移方向突变瞬间的最小行走轮输出比例
#define CHASSIS_REMOTE_SC_CENTER 2U                   // SC default center; only 2->1/3 triggers yaw preset.
#define CHASSIS_REMOTE_SC_LEFT_90 1U                  // SC 2->1: current yaw + 90 deg.
#define CHASSIS_REMOTE_SC_RIGHT_90 3U                 // SC 2->3: current yaw - 90 deg.
#define CHASSIS_REMOTE_YAW_STEP_DEG 90.0f             // SC yaw preset step.
#define CHASSIS_IDLE_DRIVE_HOLD_ENABLE 1U             // 静止时行走轮速度环 0 保持
#define CHASSIS_STOP_STEERING_RETURN_TO_FRONT_ENABLE 0U // 静止时舵轮是否回前后朝向: 1=回正, 0=保持当前角度
#define CHASSIS_DRIVE_WAIT_STEERING_ENABLE 0U         // 前进/平移时等待舵向到位后再启动行走轮
#define CHASSIS_DRIVE_WAIT_STEERING_TOLERANCE_DEG 10.0f // 舵向到位判定角度误差
#define CHASSIS_DRIVE_WAIT_STEERING_TIMEOUT_MS 300U   // 舵向等待超时,避免行走轮永久不动

typedef enum {
    CHASSIS_STEERING_HOME_WAIT_FEEDBACK = 0,
    CHASSIS_STEERING_HOME_CHECK_INITIAL_GATE,
    CHASSIS_STEERING_HOME_FIND_UNBLOCKED,
    CHASSIS_STEERING_HOME_CONFIRM_UNBLOCKED,
    CHASSIS_STEERING_HOME_SEARCH_BLOCKED,
    CHASSIS_STEERING_HOME_CONFIRM_BLOCKED,
    CHASSIS_STEERING_HOME_SETTLE_AFTER_RESET,
    CHASSIS_STEERING_HOME_DONE,
    CHASSIS_STEERING_HOME_ERROR,
} ChassisSteeringHomeState_e;

typedef struct {
    DJIMotor_Instance *motor;
    GPIO_TypeDef *photo_gate_port;
    uint16_t photo_gate_pin;
    float target_total_ecd;
    float align_angle_ref;
    ChassisSteeringHomeState_e state;
    uint32_t state_tick;
    uint8_t is_homed;
} ChassisSteeringHome_s;

static ChassisSteeringHome_s steering_home[CHASSIS_STEERING_COUNT] = {
    [CHASSIS_STEERING_LF] = {NULL, GPIOE, GPIO_PIN_14, STEERING_CHASSIS_ALIGN_TOTAL_ECD_LF, 0.0f, CHASSIS_STEERING_HOME_WAIT_FEEDBACK, 0U, 0U},
    [CHASSIS_STEERING_RF] = {NULL, GPIOE, GPIO_PIN_12, STEERING_CHASSIS_ALIGN_TOTAL_ECD_RF, 0.0f, CHASSIS_STEERING_HOME_WAIT_FEEDBACK, 0U, 0U},
    [CHASSIS_STEERING_LB] = {NULL, GPIOE, GPIO_PIN_11, STEERING_CHASSIS_ALIGN_TOTAL_ECD_LB, 0.0f, CHASSIS_STEERING_HOME_WAIT_FEEDBACK, 0U, 0U},
    [CHASSIS_STEERING_RB] = {NULL, GPIOE, GPIO_PIN_10, STEERING_CHASSIS_ALIGN_TOTAL_ECD_RB, 0.0f, CHASSIS_STEERING_HOME_WAIT_FEEDBACK, 0U, 0U},
};

static uint8_t steering_home_all_done = 0U;

/* 航向锁定 PID 控制器 */
static PID_Instance chassis_follow_pid;

/* 临时目标轮速与角度（用于某些特殊运动模型） */
static float vt_lf, vt_rf, vt_lb, vt_rb;
static float at_lf, at_rf, at_lb, at_rb;
static float chassis_translation_drive_scale = 1.0f;

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

ChassisDebug_s chassis_debug;

static void ChassisDebugUpdateHomeState(void)
{
    chassis_debug.steering_home_all_done = steering_home_all_done;

    for (uint8_t i = 0U; i < CHASSIS_STEERING_COUNT; i++) {
        chassis_debug.home_state[i] = (uint8_t)steering_home[i].state;
        chassis_debug.home_done[i] = steering_home[i].is_homed;
    }
}

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

static uint8_t ChassisRemoteYawPresetTask(uint8_t sc)
{
    static uint8_t last_sc = 0U;
    uint8_t previous_sc;
    float target_yaw;
    uint8_t triggered = 0U;

    if (sc == last_sc) {
        return 0U;
    }

    previous_sc = last_sc;
    last_sc = sc;

    if ((chassis_ctrl_cmd.imu_enable == 0U) ||
        (chassis_ctrl_cmd.Chassis_IMU_data == NULL) ||
        (chassis_ctrl_cmd.Chassis_IMU_data->online == 0U)) {
        return 0U;
    }

    if (previous_sc != CHASSIS_REMOTE_SC_CENTER) {
        return 0U;
    }

    switch (sc) {
        case CHASSIS_REMOTE_SC_LEFT_90:
            target_yaw = ChassisIMU_NormalizeDeg(chassis_ctrl_cmd.Chassis_IMU_data->Yaw +
                                                 CHASSIS_REMOTE_YAW_STEP_DEG);
            triggered = 1U;
            break;

        case CHASSIS_REMOTE_SC_RIGHT_90:
            target_yaw = ChassisIMU_NormalizeDeg(chassis_ctrl_cmd.Chassis_IMU_data->Yaw -
                                                 CHASSIS_REMOTE_YAW_STEP_DEG);
            triggered = 1U;
            break;

        default:
            break;
    }

    if (triggered != 0U) {
        chassis_ctrl_cmd.last_yaw = target_yaw;
        chassis_ctrl_cmd.target_yaw = target_yaw;
        chassis_ctrl_cmd.offset_w = 0.0f;
        ChassisIMU_ClearCorrectionPID();
    }

    return triggered;
}

static void ChassisSteeringClearPID(PID_Instance *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
}

static float ChassisSteeringNormalizeTargetTotalEcd(float target_total_ecd)
{
    while (target_total_ecd > CHASSIS_STEER_WHEEL_TOTAL_ECD_HALF_RANGE) {
        target_total_ecd -= CHASSIS_STEER_WHEEL_TOTAL_ECD_RANGE;
    }

    while (target_total_ecd < -CHASSIS_STEER_WHEEL_TOTAL_ECD_HALF_RANGE) {
        target_total_ecd += CHASSIS_STEER_WHEEL_TOTAL_ECD_RANGE;
    }

    return target_total_ecd;
}

static float ChassisSteeringWheelAngleToMotorAngle(const ChassisSteeringHome_s *home,
                                                   float wheel_angle_deg)
{
    float target_total_ecd;

    if (home == NULL) {
        return 0.0f;
    }

    target_total_ecd = home->target_total_ecd +
                       wheel_angle_deg / 360.0f * CHASSIS_STEER_WHEEL_TOTAL_ECD_RANGE;
    target_total_ecd = ChassisSteeringNormalizeTargetTotalEcd(target_total_ecd);

    return CHASSIS_STEER_ECD_TO_DEG(target_total_ecd);
}

static float ChassisSteeringWheelAngle(const ChassisSteeringHome_s *home)
{
    float motor_total_ecd;
    float wheel_total_ecd;

    if ((home == NULL) || (home->motor == NULL)) {
        return 0.0f;
    }

    motor_total_ecd = home->motor->measure.total_angle / 360.0f * CHASSIS_STEER_MOTOR_ECD_RANGE;
    wheel_total_ecd = ChassisSteeringNormalizeTargetTotalEcd(motor_total_ecd - home->target_total_ecd);

    return wheel_total_ecd / CHASSIS_STEER_WHEEL_TOTAL_ECD_RANGE * 360.0f;
}

static uint8_t ChassisSteeringWheelAngleReady(const ChassisSteeringHome_s *home,
                                              float target_wheel_angle_deg,
                                              float tolerance_deg)
{
    float current_wheel_angle_deg;
    float angle_error_deg;

    if ((home == NULL) || (home->motor == NULL) || (home->is_homed == 0U)) {
        return 0U;
    }

    current_wheel_angle_deg = ChassisSteeringWheelAngle(home);
    angle_error_deg = fabsf(ChassisIMU_DiffDeg(target_wheel_angle_deg, current_wheel_angle_deg));

    return (angle_error_deg <= tolerance_deg) ? 1U : 0U;
}

static uint8_t ChassisSteeringPhotoGateBlocked(const ChassisSteeringHome_s *home)
{
    if ((home == NULL) || (home->photo_gate_port == NULL)) {
        return 0U;
    }

    return (HAL_GPIO_ReadPin(home->photo_gate_port, home->photo_gate_pin) ==
            CHASSIS_STEERING_PHOTO_GATE_BLOCKED)
               ? 1U
               : 0U;
}

static void ChassisSteeringEnterHomeState(ChassisSteeringHome_s *home,
                                          ChassisSteeringHomeState_e state)
{
    if (home == NULL) {
        return;
    }

    home->state = state;
    home->state_tick = HAL_GetTick();
}

/**
 * @brief 根据归零状态对象反查舵轮编号，用于选择对应舵电机参数。
 */
static uint8_t ChassisSteeringIndexFromHome(const ChassisSteeringHome_s *home)
{
    if (home == &steering_home[CHASSIS_STEERING_RF]) {
        return CHASSIS_STEERING_RF;
    }
    if (home == &steering_home[CHASSIS_STEERING_LB]) {
        return CHASSIS_STEERING_LB;
    }
    if (home == &steering_home[CHASSIS_STEERING_RB]) {
        return CHASSIS_STEERING_RB;
    }

    return CHASSIS_STEERING_LF;
}

static void ChassisSteeringSetHomeSpeedPID(ChassisSteeringHome_s *home)
{
    PID_Instance *speed_pid;

    if ((home == NULL) || (home->motor == NULL)) {
        return;
    }

    speed_pid = &home->motor->motor_controller.speed_PID;
    speed_pid->Kp = CHASSIS_STEERING_HOME_SPEED_KP;
    speed_pid->Ki = CHASSIS_STEERING_HOME_SPEED_KI;
    speed_pid->Kd = CHASSIS_STEERING_HOME_SPEED_KD;
    speed_pid->Improve = PID_OutputFilter;
    speed_pid->IntegralLimit = 0.0f;
    speed_pid->MaxOut = CHASSIS_STEERING_HOME_SPEED_MAX_OUT;
    speed_pid->Output_LPF_RC = CHASSIS_STEERING_HOME_SPEED_LPF_RC;
    ChassisSteeringClearPID(speed_pid);
}

static void ChassisSteeringSetNormalSpeedPID(ChassisSteeringHome_s *home)
{
    PID_Instance *speed_pid;
    float kp = CHASSIS_STEERING_SPEED_KP_LF;
    float ki = CHASSIS_STEERING_SPEED_KI_LF;
    float kd = CHASSIS_STEERING_SPEED_KD_LF;
    float integral_limit = CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_LF;
    float output_lpf_rc = CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_LF;
    uint8_t index;

    if ((home == NULL) || (home->motor == NULL)) {
        return;
    }

    index = ChassisSteeringIndexFromHome(home);
    if (index == CHASSIS_STEERING_RF) {
        kp = CHASSIS_STEERING_SPEED_KP_RF;
        ki = CHASSIS_STEERING_SPEED_KI_RF;
        kd = CHASSIS_STEERING_SPEED_KD_RF;
        integral_limit = CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_RF;
        output_lpf_rc = CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_RF;
    } else if (index == CHASSIS_STEERING_LB) {
        kp = CHASSIS_STEERING_SPEED_KP_LB;
        ki = CHASSIS_STEERING_SPEED_KI_LB;
        kd = CHASSIS_STEERING_SPEED_KD_LB;
        integral_limit = CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_LB;
        output_lpf_rc = CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_LB;
    } else if (index == CHASSIS_STEERING_RB) {
        kp = CHASSIS_STEERING_SPEED_KP_RB;
        ki = CHASSIS_STEERING_SPEED_KI_RB;
        kd = CHASSIS_STEERING_SPEED_KD_RB;
        integral_limit = CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_RB;
        output_lpf_rc = CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_RB;
    }

    speed_pid = &home->motor->motor_controller.speed_PID;
    speed_pid->Kp = kp;
    speed_pid->Ki = ki;
    speed_pid->Kd = kd;
    speed_pid->Improve = PID_Integral_Limit | PID_Derivative_On_Measurement |
                         PID_ChangingIntegrationRate | PID_OutputFilter;
    speed_pid->IntegralLimit = integral_limit;
    speed_pid->MaxOut = CHASSIS_STEERING_SPEED_MAX_OUT;
    speed_pid->Output_LPF_RC = output_lpf_rc;
    ChassisSteeringClearPID(speed_pid);
}

static void ChassisSteeringSetHomeSpeedRef(ChassisSteeringHome_s *home, float speed_ref)
{
    if ((home == NULL) || (home->motor == NULL)) {
        return;
    }

    DJIMotorEnable(home->motor);
    DJIMotorOuterLoop(home->motor, SPEED_LOOP);
    DJIMotorSetRef(home->motor, speed_ref);
}

static void ChassisSteeringStopMotor(ChassisSteeringHome_s *home)
{
    if ((home == NULL) || (home->motor == NULL)) {
        return;
    }

    DJIMotorSetRef(home->motor, 0.0f);
    DJIMotorStop(home->motor);
}

/**
 * @brief 判断指定舵轮当前是否参与临时测试。
 */
static uint8_t ChassisSteeringTestEnabled(uint8_t index)
{
    if (index >= CHASSIS_STEERING_COUNT) {
        return 0U;
    }

#if CHASSIS_SINGLE_SWERVE_TEST_ENABLE
    if (index != CHASSIS_SINGLE_SWERVE_TEST_INDEX) {
        return 0U;
    }
#endif

#if !CHASSIS_LB_SWERVE_TEST_ENABLE
    if (index == CHASSIS_STEERING_LB) {
        return 0U;
    }
#endif

    return 1U;
}

static void ChassisSteeringSetAlignAngleRef(ChassisSteeringHome_s *home)
{
    if ((home == NULL) || (home->motor == NULL)) {
        return;
    }

#if CHASSIS_STEERING_ALIGN_ENABLE
    DJIMotorEnable(home->motor);
    DJIMotorOuterLoop(home->motor, ANGLE_LOOP);
    DJIMotorSetRef(home->motor, home->align_angle_ref);
#else
    ChassisSteeringStopMotor(home);
#endif
}

static void ChassisSteeringStartHome(ChassisSteeringHome_s *home)
{
    if (home == NULL) {
        return;
    }

    home->align_angle_ref = 0.0f;
    home->is_homed = 0U;

    if ((home->motor == NULL) || (home->motor->feedback_initialized == 0U)) {
        ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_WAIT_FEEDBACK);
        return;
    }

    ChassisSteeringSetHomeSpeedPID(home);
    ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_CHECK_INITIAL_GATE);
}

static void ChassisSteeringStartHomeAll(void)
{
    uint8_t has_enabled_steering = 0U;

    steering_home[CHASSIS_STEERING_LF].motor = motor_steering_lf;
    steering_home[CHASSIS_STEERING_RF].motor = motor_steering_rf;
    steering_home[CHASSIS_STEERING_LB].motor = motor_steering_lb;
    steering_home[CHASSIS_STEERING_RB].motor = motor_steering_rb;

    for (uint8_t i = 0U; i < CHASSIS_STEERING_COUNT; i++) {
        steering_home[i].align_angle_ref = 0.0f;
        steering_home[i].is_homed = 0U;
        ChassisSteeringEnterHomeState(&steering_home[i], CHASSIS_STEERING_HOME_WAIT_FEEDBACK);
        ChassisSteeringStopMotor(&steering_home[i]);
    }

    for (uint8_t i = 0U; i < CHASSIS_STEERING_COUNT; i++) {
        if (ChassisSteeringTestEnabled(i) != 0U) {
            has_enabled_steering = 1U;
            ChassisSteeringStartHome(&steering_home[i]);
        } else {
            ChassisSteeringStopMotor(&steering_home[i]);
        }
    }

    steering_home_all_done = (has_enabled_steering != 0U) ? 0U : 1U;
}

static void ChassisSteeringHomeTask(ChassisSteeringHome_s *home)
{
    uint32_t now = HAL_GetTick();

    if (home == NULL) {
        return;
    }

    if ((home->motor == NULL) || (home->motor->feedback_initialized == 0U)) {
        home->is_homed = 0U;
        ChassisSteeringStopMotor(home);
        ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_WAIT_FEEDBACK);
        return;
    }

    switch (home->state) {
        case CHASSIS_STEERING_HOME_WAIT_FEEDBACK:
            ChassisSteeringSetHomeSpeedPID(home);
            ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_CHECK_INITIAL_GATE);
            break;

        case CHASSIS_STEERING_HOME_CHECK_INITIAL_GATE:
            home->is_homed = 0U;
            ChassisSteeringSetHomeSpeedPID(home);
            if (ChassisSteeringPhotoGateBlocked(home) != 0U) {
                ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_FIND_UNBLOCKED);
            } else {
                ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_SEARCH_BLOCKED);
            }
            break;

        case CHASSIS_STEERING_HOME_FIND_UNBLOCKED:
            ChassisSteeringSetHomeSpeedRef(home, CHASSIS_STEERING_HOME_SPEED_REF);
            if (ChassisSteeringPhotoGateBlocked(home) == 0U) {
                ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_CONFIRM_UNBLOCKED);
            } else if ((now - home->state_tick) >= CHASSIS_STEERING_HOME_TIMEOUT_MS) {
                ChassisSteeringStopMotor(home);
                ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_ERROR);
            }
            break;

        case CHASSIS_STEERING_HOME_CONFIRM_UNBLOCKED:
            ChassisSteeringSetHomeSpeedRef(home, CHASSIS_STEERING_HOME_SPEED_REF);
            if (ChassisSteeringPhotoGateBlocked(home) != 0U) {
                ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_FIND_UNBLOCKED);
            } else if ((now - home->state_tick) >= CHASSIS_STEERING_HOME_UNBLOCK_CONFIRM_MS) {
                ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_SEARCH_BLOCKED);
            }
            break;

        case CHASSIS_STEERING_HOME_SEARCH_BLOCKED:
            ChassisSteeringSetHomeSpeedRef(home, CHASSIS_STEERING_HOME_SPEED_REF);
            if (ChassisSteeringPhotoGateBlocked(home) != 0U) {
                ChassisSteeringStopMotor(home);
                ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_CONFIRM_BLOCKED);
            } else if ((now - home->state_tick) >= CHASSIS_STEERING_HOME_TIMEOUT_MS) {
                ChassisSteeringStopMotor(home);
                ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_ERROR);
            }
            break;

        case CHASSIS_STEERING_HOME_CONFIRM_BLOCKED:
            ChassisSteeringStopMotor(home);
            if (ChassisSteeringPhotoGateBlocked(home) == 0U) {
                ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_SEARCH_BLOCKED);
            } else if ((now - home->state_tick) >= CHASSIS_STEERING_HOME_CONFIRM_MS) {
                DJIMotorReset(home->motor);
                home->motor->measure.total_angle = 0.0f;
                home->align_angle_ref =
                    ChassisSteeringWheelAngleToMotorAngle(home, 0.0f);
                ChassisSteeringSetNormalSpeedPID(home);
                ChassisSteeringClearPID(&home->motor->motor_controller.angle_PID);
                home->is_homed = 1U;
                ChassisSteeringEnterHomeState(home,
                                              CHASSIS_STEERING_HOME_SETTLE_AFTER_RESET);
            }
            break;

        case CHASSIS_STEERING_HOME_SETTLE_AFTER_RESET:
            ChassisSteeringStopMotor(home);
            if ((now - home->state_tick) >= CHASSIS_STEERING_HOME_SETTLE_MS) {
                ChassisSteeringEnterHomeState(home, CHASSIS_STEERING_HOME_DONE);
            }
            break;

        case CHASSIS_STEERING_HOME_DONE:
            ChassisSteeringSetAlignAngleRef(home);
            break;

        case CHASSIS_STEERING_HOME_ERROR:
        default:
            home->is_homed = 0U;
            ChassisSteeringStopMotor(home);
            break;
    }
}

static void ChassisSteeringHomeTaskAll(void)
{
    uint8_t all_done = 1U;

    if (steering_home_all_done != 0U) {
        return;
    }

    for (uint8_t i = 0U; i < CHASSIS_STEERING_COUNT; i++) {
        if (ChassisSteeringTestEnabled(i) != 0U) {
            ChassisSteeringHomeTask(&steering_home[i]);

            if ((steering_home[i].state == CHASSIS_STEERING_HOME_DONE) &&
                (steering_home[i].is_homed != 0U)) {
                ChassisSteeringSetAlignAngleRef(&steering_home[i]);
            } else {
                all_done = 0U;
            }
        } else {
            ChassisSteeringStopMotor(&steering_home[i]);
        }
    }

    steering_home_all_done = all_done;
    ChassisDebugUpdateHomeState();
}

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

static void ChassisStopDriveMotors(void)
{
    ChassisSetDriveMotorRef(motor_lf, 0.0f, CHASSIS_ID3_M3508_SPEED_DEADBAND);
    ChassisSetDriveMotorRef(motor_rf, 0.0f, CHASSIS_ID1_M3508_SPEED_DEADBAND);
    ChassisSetDriveMotorRef(motor_lb, 0.0f, CHASSIS_ID4_M3508_SPEED_DEADBAND);
    ChassisSetDriveMotorRef(motor_rb, 0.0f, CHASSIS_ID2_M3508_SPEED_DEADBAND);
}

static void ChassisHoldDriveMotor(DJIMotor_Instance *motor)
{
    if (motor == NULL) {
        return;
    }

    DJIMotorEnable(motor);
    DJIMotorOuterLoop(motor, SPEED_LOOP);
    DJIMotorSetRef(motor, 0.0f);
}

static void ChassisHoldDriveMotors(void)
{
    ChassisHoldDriveMotor(motor_lf);
    ChassisHoldDriveMotor(motor_rf);
    ChassisHoldDriveMotor(motor_lb);
    ChassisHoldDriveMotor(motor_rb);
}

static void ChassisStopSteeringMotors(void)
{
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_LF]);
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_RF]);
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_LB]);
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_RB]);
}

static uint8_t ChassisRemoteIsOnline(void)
{
    uint32_t now;

    if ((remote_dev == NULL) || (remote_data == NULL)) {
        return 0U;
    }

    if (remote_dev->last_update_time == 0U) {
        return 0U;
    }

    now = HAL_GetTick();
    if ((now - remote_dev->last_update_time) > CHASSIS_REMOTE_TIMEOUT_MS) {
        return 0U;
    }

    return 1U;
}

#if CHASSIS_TRANSLATION_DIR_CHANGE_ENABLE
static float ChassisTranslationDirectionChangeScale(float vx,
                                                    float vy,
                                                    float raw_translation_norm,
                                                    float translation_drive_start)
{
    static uint8_t last_translation_active = 0U;
    static float last_translation_dir_deg = 0.0f;
    static uint8_t dir_change_active = 0U;
    static uint32_t dir_change_start_tick = 0U;
    uint8_t translation_active =
        (raw_translation_norm > translation_drive_start) ? 1U : 0U;
    float current_dir_deg = 0.0f;
    float dir_delta_deg = 0.0f;
    float scale = 1.0f;

    if (translation_active == 0U) {
        last_translation_active = 0U;
        dir_change_active = 0U;
        chassis_debug.translation_dir_change_active = 0U;
        chassis_debug.translation_dir_deg = 0.0f;
        chassis_debug.translation_dir_delta_deg = 0.0f;
        chassis_debug.translation_dir_change_scale = 1.0f;
        return 1.0f;
    }

    current_dir_deg = atan2f(vy, vx) * RAD_2_DEGREE;
    chassis_debug.translation_dir_deg = current_dir_deg;

    if (last_translation_active != 0U) {
        dir_delta_deg = fabsf(ChassisIMU_DiffDeg(current_dir_deg, last_translation_dir_deg));
        if (dir_delta_deg > CHASSIS_TRANSLATION_DIR_CHANGE_TRIGGER_DEG) {
            dir_change_active = 1U;
            dir_change_start_tick = HAL_GetTick();
        }
    }

    last_translation_dir_deg = current_dir_deg;
    last_translation_active = 1U;
    chassis_debug.translation_dir_delta_deg = dir_delta_deg;

    if (dir_change_active != 0U) {
        uint32_t elapsed_ms = HAL_GetTick() - dir_change_start_tick;

        if (elapsed_ms >= CHASSIS_TRANSLATION_DIR_CHANGE_TIME_MS) {
            dir_change_active = 0U;
        } else {
            float ratio = (float)elapsed_ms / (float)CHASSIS_TRANSLATION_DIR_CHANGE_TIME_MS;
            scale = CHASSIS_TRANSLATION_DIR_CHANGE_MIN_SCALE +
                    (1.0f - CHASSIS_TRANSLATION_DIR_CHANGE_MIN_SCALE) * ratio;
        }
    }

    chassis_debug.translation_dir_change_active = dir_change_active;
    chassis_debug.translation_dir_change_scale = scale;

    return scale;
}
#endif

/**
 * @brief 在注册舵向电机前写入该舵轮独立的角度环、速度环参数和公共电流环参数。
 */
static void ChassisSteeringApplyInitPID(Motor_Init_Config_s *config, uint8_t index)
{
    if (config == NULL) {
        return;
    }

    config->controller_param_init_config.angle_PID.Kp = CHASSIS_STEERING_ANGLE_KP_LF;
    config->controller_param_init_config.angle_PID.Ki = CHASSIS_STEERING_ANGLE_KI_LF;
    config->controller_param_init_config.angle_PID.Kd = CHASSIS_STEERING_ANGLE_KD_LF;
    config->controller_param_init_config.angle_PID.IntegralLimit = CHASSIS_STEERING_ANGLE_INTEGRAL_LIMIT_LF;
    config->controller_param_init_config.speed_PID.Kp = CHASSIS_STEERING_SPEED_KP_LF;
    config->controller_param_init_config.speed_PID.Ki = CHASSIS_STEERING_SPEED_KI_LF;
    config->controller_param_init_config.speed_PID.Kd = CHASSIS_STEERING_SPEED_KD_LF;
    config->controller_param_init_config.speed_PID.IntegralLimit = CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_LF;
    config->controller_param_init_config.speed_PID.Output_LPF_RC = CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_LF;
    config->controller_param_init_config.current_PID.Kp = CHASSIS_STEERING_CURRENT_KP;
    config->controller_param_init_config.current_PID.Ki = CHASSIS_STEERING_CURRENT_KI;
    config->controller_param_init_config.current_PID.Kd = CHASSIS_STEERING_CURRENT_KD;
    config->controller_param_init_config.current_PID.IntegralLimit = CHASSIS_STEERING_CURRENT_INTEGRAL_LIMIT;
    config->controller_param_init_config.current_PID.MaxOut = CHASSIS_STEERING_CURRENT_MAX_OUT;

    if (index == CHASSIS_STEERING_RF) {
        config->controller_param_init_config.angle_PID.Kp = CHASSIS_STEERING_ANGLE_KP_RF;
        config->controller_param_init_config.angle_PID.Ki = CHASSIS_STEERING_ANGLE_KI_RF;
        config->controller_param_init_config.angle_PID.Kd = CHASSIS_STEERING_ANGLE_KD_RF;
        config->controller_param_init_config.angle_PID.IntegralLimit = CHASSIS_STEERING_ANGLE_INTEGRAL_LIMIT_RF;
        config->controller_param_init_config.speed_PID.Kp = CHASSIS_STEERING_SPEED_KP_RF;
        config->controller_param_init_config.speed_PID.Ki = CHASSIS_STEERING_SPEED_KI_RF;
        config->controller_param_init_config.speed_PID.Kd = CHASSIS_STEERING_SPEED_KD_RF;
        config->controller_param_init_config.speed_PID.IntegralLimit = CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_RF;
        config->controller_param_init_config.speed_PID.Output_LPF_RC = CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_RF;
    } else if (index == CHASSIS_STEERING_LB) {
        config->controller_param_init_config.angle_PID.Kp = CHASSIS_STEERING_ANGLE_KP_LB;
        config->controller_param_init_config.angle_PID.Ki = CHASSIS_STEERING_ANGLE_KI_LB;
        config->controller_param_init_config.angle_PID.Kd = CHASSIS_STEERING_ANGLE_KD_LB;
        config->controller_param_init_config.angle_PID.IntegralLimit = CHASSIS_STEERING_ANGLE_INTEGRAL_LIMIT_LB;
        config->controller_param_init_config.speed_PID.Kp = CHASSIS_STEERING_SPEED_KP_LB;
        config->controller_param_init_config.speed_PID.Ki = CHASSIS_STEERING_SPEED_KI_LB;
        config->controller_param_init_config.speed_PID.Kd = CHASSIS_STEERING_SPEED_KD_LB;
        config->controller_param_init_config.speed_PID.IntegralLimit = CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_LB;
        config->controller_param_init_config.speed_PID.Output_LPF_RC = CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_LB;
    } else if (index == CHASSIS_STEERING_RB) {
        config->controller_param_init_config.angle_PID.Kp = CHASSIS_STEERING_ANGLE_KP_RB;
        config->controller_param_init_config.angle_PID.Ki = CHASSIS_STEERING_ANGLE_KI_RB;
        config->controller_param_init_config.angle_PID.Kd = CHASSIS_STEERING_ANGLE_KD_RB;
        config->controller_param_init_config.angle_PID.IntegralLimit = CHASSIS_STEERING_ANGLE_INTEGRAL_LIMIT_RB;
        config->controller_param_init_config.speed_PID.Kp = CHASSIS_STEERING_SPEED_KP_RB;
        config->controller_param_init_config.speed_PID.Ki = CHASSIS_STEERING_SPEED_KI_RB;
        config->controller_param_init_config.speed_PID.Kd = CHASSIS_STEERING_SPEED_KD_RB;
        config->controller_param_init_config.speed_PID.IntegralLimit = CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_RB;
        config->controller_param_init_config.speed_PID.Output_LPF_RC = CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_RB;
    }
}

void ChassisInit()
{
    // 四个行走电机参数基本一致，主要区别是 CAN ID 和电机方向。
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.fdcan_handle   = &hfdcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 3.5, // 3
                .Ki            = 0.2, // 0.5
                .Kd            = 0.001,   // 0
                .IntegralLimit = 3000,//5000
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 25000,
            },
            .current_PID = {
                .Kp            = 1.2, // 1
                .Ki            = 0.01,   // 0
                .Kd            = 0,
                .IntegralLimit = 3000,//3000
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 25000,
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
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.controller_param_init_config.speed_PID.DeadBand   = CHASSIS_ID3_M3508_SPEED_DEADBAND;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis_motor_config.controller_param_init_config.speed_PID.DeadBand   = CHASSIS_ID1_M3508_SPEED_DEADBAND;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.controller_param_init_config.speed_PID.DeadBand   = CHASSIS_ID4_M3508_SPEED_DEADBAND;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.controller_param_init_config.speed_PID.DeadBand   = CHASSIS_ID2_M3508_SPEED_DEADBAND;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);


    // 3508 转向电机初始化。
    Motor_Init_Config_s chassis_motor_steering_config = {
        .can_init_config.fdcan_handle   = &hfdcan2,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = CHASSIS_STEERING_ANGLE_KP_LF,
                .Ki                = CHASSIS_STEERING_ANGLE_KI_LF,
                .Kd                = CHASSIS_STEERING_ANGLE_KD_LF,
                .CoefA             = CHASSIS_STEERING_ANGLE_COEF_A,
                .CoefB             = CHASSIS_STEERING_ANGLE_COEF_B,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .IntegralLimit     = CHASSIS_STEERING_ANGLE_INTEGRAL_LIMIT_LF,
                .MaxOut            = CHASSIS_STEERING_ANGLE_MAX_OUT,
                .Derivative_LPF_RC = CHASSIS_STEERING_ANGLE_DERIVATIVE_LPF_RC,
                .DeadBand          = CHASSIS_STEERING_ANGLE_DEADBAND,
            },
            .speed_PID = {
                .Kp            = CHASSIS_STEERING_SPEED_KP_LF,
                .Ki            = CHASSIS_STEERING_SPEED_KI_LF,
                .Kd            = CHASSIS_STEERING_SPEED_KD_LF,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
                .IntegralLimit = CHASSIS_STEERING_SPEED_INTEGRAL_LIMIT_LF,
                .MaxOut        = CHASSIS_STEERING_SPEED_MAX_OUT,
                .Output_LPF_RC = CHASSIS_STEERING_SPEED_OUTPUT_LPF_RC_LF,
            },
            .current_PID = {
                .Kp            = CHASSIS_STEERING_CURRENT_KP,
                .Ki            = CHASSIS_STEERING_CURRENT_KI,
                .Kd            = CHASSIS_STEERING_CURRENT_KD,
                .IntegralLimit = CHASSIS_STEERING_CURRENT_INTEGRAL_LIMIT,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = CHASSIS_STEERING_CURRENT_MAX_OUT,
            },

        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .output_reverse_flag   = MOTOR_DIRECTION_NORMAL,
            .angle_mode            = MOTOR_ANGLE_MODE_TOTAL,
            .feedforward_flag      = FEEDFORWARD_NONE,
        },
        .motor_type = M3508,
    };
    chassis_motor_steering_config.can_init_config.tx_id = 7;
    ChassisSteeringApplyInitPID(&chassis_motor_steering_config, CHASSIS_STEERING_LF);
    motor_steering_lf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 8;
    ChassisSteeringApplyInitPID(&chassis_motor_steering_config, CHASSIS_STEERING_RF);
    motor_steering_rf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 5;
    ChassisSteeringApplyInitPID(&chassis_motor_steering_config, CHASSIS_STEERING_LB);
    motor_steering_lb                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 6;
    ChassisSteeringApplyInitPID(&chassis_motor_steering_config, CHASSIS_STEERING_RB);
    motor_steering_rb                                   = DJIMotorInit(&chassis_motor_steering_config);

            PID_Init_Config_s chassis_follow_pid_conf = {
        .Kp                = 800, // 6
        .Ki                = 0.5f,
        .Kd                = 17, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ErrorHandle,
        .IntegralLimit     = 5000, // 200
        .MaxOut            = 25000,
        .Derivative_LPF_RC = 0.01, // 0.01
    };

    PIDInit(&chassis_follow_pid, &chassis_follow_pid_conf);
    ChassisIMU_Init();
    ChassisIMU_Enable(CHASSIS_IMU_CORRECTION_ENABLE);
    ChassisIMU_SetCorrectMode(IMU_CORRECT_STRAIGHT);
    ChassisSteeringStartHomeAll();

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

    return offset * CHASSIS_IMU_CORRECTION_SIGN;
}

#if CHASSIS_IDLE_YAW_CORRECTION_HOLD_ENABLE
static float ChassisIMU_GetCorrectionYawError(void)
{
    float current_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;

    if (chassis_ctrl_cmd.correct_mode == IMU_CORRECT_ROTATION) {
        return ChassisIMU_DiffDeg(chassis_ctrl_cmd.target_yaw, current_yaw);
    }

    return ChassisIMU_DiffDeg(chassis_ctrl_cmd.last_yaw, current_yaw);
}
#endif

/**
 * @brief 舵轮底盘运动学解算。
 * @param vx 前后方向线速度指令。
 * @param vy 左右方向线速度指令。
 * @param vw 角速度指令。
 * @note 当前使用4轮舵轮模式：3 号轮为左前轮，1 号轮为左后轮，2 号轮为右后轮，4 号轮为右前轮。
 */
void SteeringWheelKinematics(float vx, float vy, float vw)
{
    float chassis_vx = vx;
    float chassis_vy = vy;
    float chassis_vw = vw;
    static uint8_t first_run_kinematics = 1;
#if CHASSIS_IDLE_YAW_CORRECTION_HOLD_ENABLE
    static uint8_t idle_yaw_correction_hold = 0U;
#endif
    static uint8_t manual_rotate_active = 0U;
    static uint8_t rotate_release_settle_active = 0U;
    static uint32_t rotate_release_settle_start_tick = 0U;
    static uint8_t drive_wait_steering_active = 0U;
    static uint32_t drive_wait_steering_start_tick = 0U;
    float offset_lf = 0.0f, offset_rf = 0.0f, offset_lb = 0.0f, offset_rb = 0.0f;
    float at_lf_last = 0.0f, at_rf_last = 0.0f, at_lb_last = 0.0f, at_rb_last = 0.0f;
    uint8_t manual_idle = ((vx == 0.0f) && (vy == 0.0f) && (vw == 0.0f)) ? 1U : 0U;
    uint8_t stop_align_ready = 0U;
    uint8_t drive_wait_steering_ready = 1U;
    uint8_t drive_wait_steering_needed = 0U;
    uint8_t rotate_imu_hold_active = 0U;
    float current_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;
#if CHASSIS_IDLE_YAW_CORRECTION_HOLD_ENABLE
    float idle_yaw_error = 0.0f;
#endif

    if (steering_home_all_done == 0U) {
        chassis_debug.stop_reason = 2U;
        chassis_debug.cmd_vw_corrected = 0.0f;
        chassis_debug.imu_offset_w = 0.0f;
        chassis_debug.manual_rotate_active = 0U;
        chassis_debug.rotate_release_settle_active = 0U;
        ChassisDebugUpdateHomeState();
        ChassisStopDriveMotors();
        return;
    }

#if !CHASSIS_STEERING_ALIGN_ENABLE
    chassis_debug.stop_reason = 2U;
    chassis_debug.cmd_vw_corrected = 0.0f;
    chassis_debug.imu_offset_w = 0.0f;
    chassis_debug.manual_rotate_active = 0U;
    chassis_debug.rotate_release_settle_active = 0U;
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_LF]);
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_RF]);
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_LB]);
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_RB]);
    ChassisStopDriveMotors();
    return;
#endif

    chassis_debug.stop_reason = 0U;

    at_rf_last = ChassisSteeringWheelAngle(&steering_home[CHASSIS_STEERING_RF]);
#if !CHASSIS_SINGLE_SWERVE_TEST_ENABLE
    at_lf_last = ChassisSteeringWheelAngle(&steering_home[CHASSIS_STEERING_LF]);
#if CHASSIS_LB_SWERVE_TEST_ENABLE
    at_lb_last = ChassisSteeringWheelAngle(&steering_home[CHASSIS_STEERING_LB]);
#endif
    at_rb_last = ChassisSteeringWheelAngle(&steering_home[CHASSIS_STEERING_RB]);
#endif

    if (first_run_kinematics) {
        chassis_ctrl_cmd.last_yaw = current_yaw;
        chassis_ctrl_cmd.target_yaw = current_yaw;
        first_run_kinematics = 0;
    }

    if (fabsf(vw) >= CHASSIS_MANUAL_ROTATE_ENTER_VW) {
        manual_rotate_active = 1U;
        rotate_release_settle_active = 0U;
    } else if ((manual_rotate_active != 0U) &&
               (fabsf(vw) <= CHASSIS_MANUAL_ROTATE_EXIT_VW)) {
        manual_rotate_active = 0U;
        rotate_release_settle_active = CHASSIS_ROTATE_RELEASE_SETTLE_ENABLE;
        rotate_release_settle_start_tick = HAL_GetTick();
    }

    if (rotate_release_settle_active != 0U) {
        if ((HAL_GetTick() - rotate_release_settle_start_tick) >=
            CHASSIS_ROTATE_RELEASE_SETTLE_MS) {
            rotate_release_settle_active = 0U;
            chassis_ctrl_cmd.last_yaw = current_yaw;
            chassis_ctrl_cmd.target_yaw = current_yaw;
            ChassisIMU_ClearCorrectionPID();
        }
    }

    rotate_imu_hold_active =
        ((manual_rotate_active != 0U) || (rotate_release_settle_active != 0U)) ? 1U : 0U;

    if (rotate_imu_hold_active != 0U) {
        chassis_ctrl_cmd.last_yaw = current_yaw;
        chassis_ctrl_cmd.target_yaw = current_yaw;
        chassis_ctrl_cmd.offset_w = 0.0f;
#if CHASSIS_IDLE_YAW_CORRECTION_HOLD_ENABLE
        idle_yaw_correction_hold = 0U;
#endif
        ChassisIMU_ClearCorrectionPID();
    }

    chassis_debug.manual_rotate_active = manual_rotate_active;
    chassis_debug.rotate_release_settle_active = rotate_release_settle_active;

#if CHASSIS_IDLE_YAW_CORRECTION_HOLD_ENABLE
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

    if (rotate_imu_hold_active != 0U) {
        chassis_ctrl_cmd.offset_w = 0.0f;
        chassis_vw = vw;
    } else if (idle_yaw_correction_hold != 0U) {
        chassis_ctrl_cmd.offset_w = 0.0f;
        chassis_vw = 0.0f;
    } else {
        chassis_ctrl_cmd.offset_w = UpdateIMUCorrection(vw);
        chassis_vw = vw + chassis_ctrl_cmd.offset_w;
    }
#else
    if (rotate_imu_hold_active != 0U) {
        chassis_ctrl_cmd.offset_w = 0.0f;
        chassis_vw = vw;
    } else {
        chassis_ctrl_cmd.offset_w = UpdateIMUCorrection(vw);
        chassis_vw = vw + chassis_ctrl_cmd.offset_w;
    }
#endif

    chassis_debug.cmd_vw_corrected = chassis_vw;
    chassis_debug.imu_offset_w = chassis_ctrl_cmd.offset_w;

    stop_align_ready = (manual_idle != 0U && fabsf(chassis_vw) < CHASSIS_STOP_STEERING_ALIGN_VW_DEADBAND) ? 1U : 0U;
    if (stop_align_ready != 0U) {
        chassis_ctrl_cmd.offset_w = 0.0f;
        chassis_vw = 0.0f;
        chassis_debug.cmd_vw_corrected = 0.0f;
        chassis_debug.imu_offset_w = 0.0f;
    }

    float w = chassis_vw;
    float temp_x = chassis_vx - w;
    float temp_y = chassis_vy + w;

    vt_lf = sqrtf(temp_x * temp_x + temp_y * temp_y);
    offset_lf = atan2f(temp_y, temp_x) * RAD_2_DEGREE;

    temp_y = chassis_vy - w;
    vt_lb = sqrtf(temp_x * temp_x + temp_y * temp_y);
    offset_lb = atan2f(temp_y, temp_x) * RAD_2_DEGREE;

    temp_x = chassis_vx + w;
    temp_y = chassis_vy + w;
    vt_rf = sqrtf(temp_x * temp_x + temp_y * temp_y);
    offset_rf = atan2f(temp_y, temp_x) * RAD_2_DEGREE;

    temp_y = chassis_vy - w;
    vt_rb = sqrtf(temp_x * temp_x + temp_y * temp_y);
    offset_rb = atan2f(temp_y, temp_x) * RAD_2_DEGREE;

    at_lf = offset_lf;
    at_rf = offset_rf;
    at_lb = offset_lb;
    at_rb = offset_rb;

#if !CHASSIS_STOP_STEERING_RETURN_TO_FRONT_ENABLE
    if (stop_align_ready != 0U) {
        at_rf = at_rf_last;
#if !CHASSIS_SINGLE_SWERVE_TEST_ENABLE
        at_lf = at_lf_last;
#if CHASSIS_LB_SWERVE_TEST_ENABLE
        at_lb = at_lb_last;
#endif
        at_rb = at_rb_last;
#endif
    }
#endif

    ANGLE_LIMIT_360_TO_180_ABS(at_lf);
    ANGLE_LIMIT_360_TO_180_ABS(at_rf);
    ANGLE_LIMIT_360_TO_180_ABS(at_lb);
    ANGLE_LIMIT_360_TO_180_ABS(at_rb);

    if (stop_align_ready == 0U) {
        MinmizeRotation(&at_rf, &at_rf_last, &vt_rf);
#if !CHASSIS_SINGLE_SWERVE_TEST_ENABLE
        MinmizeRotation(&at_lf, &at_lf_last, &vt_lf);
#if CHASSIS_LB_SWERVE_TEST_ENABLE
        MinmizeRotation(&at_lb, &at_lb_last, &vt_lb);
#endif
        MinmizeRotation(&at_rb, &at_rb_last, &vt_rb);
#endif
    }

    if (chassis_translation_drive_scale < 1.0f) {
        vt_lf *= chassis_translation_drive_scale;
        vt_rf *= chassis_translation_drive_scale;
        vt_lb *= chassis_translation_drive_scale;
        vt_rb *= chassis_translation_drive_scale;
    }

#if CHASSIS_DRIVE_WAIT_STEERING_ENABLE
    drive_wait_steering_needed =
        (stop_align_ready == 0U && ((fabsf(chassis_vx) > 0.0f) || (fabsf(chassis_vy) > 0.0f))) ? 1U : 0U;

    if (drive_wait_steering_needed != 0U) {
        drive_wait_steering_ready =
            ChassisSteeringWheelAngleReady(&steering_home[CHASSIS_STEERING_RF],
                                           at_rf,
                                           CHASSIS_DRIVE_WAIT_STEERING_TOLERANCE_DEG);
#if !CHASSIS_SINGLE_SWERVE_TEST_ENABLE
        drive_wait_steering_ready &=
            ChassisSteeringWheelAngleReady(&steering_home[CHASSIS_STEERING_LF],
                                           at_lf,
                                           CHASSIS_DRIVE_WAIT_STEERING_TOLERANCE_DEG);
#if CHASSIS_LB_SWERVE_TEST_ENABLE
        drive_wait_steering_ready &=
            ChassisSteeringWheelAngleReady(&steering_home[CHASSIS_STEERING_LB],
                                           at_lb,
                                           CHASSIS_DRIVE_WAIT_STEERING_TOLERANCE_DEG);
#endif
        drive_wait_steering_ready &=
            ChassisSteeringWheelAngleReady(&steering_home[CHASSIS_STEERING_RB],
                                           at_rb,
                                           CHASSIS_DRIVE_WAIT_STEERING_TOLERANCE_DEG);
#endif

        if (drive_wait_steering_ready == 0U) {
            if (drive_wait_steering_active == 0U) {
                drive_wait_steering_active = 1U;
                drive_wait_steering_start_tick = HAL_GetTick();
            } else if ((HAL_GetTick() - drive_wait_steering_start_tick) >=
                       CHASSIS_DRIVE_WAIT_STEERING_TIMEOUT_MS) {
                drive_wait_steering_ready = 1U;
            }
        } else {
            drive_wait_steering_active = 0U;
        }
    } else {
        drive_wait_steering_active = 0U;
        drive_wait_steering_ready = 1U;
    }
#endif

    at_rf = ChassisSteeringWheelAngleToMotorAngle(&steering_home[CHASSIS_STEERING_RF], at_rf);
#if !CHASSIS_SINGLE_SWERVE_TEST_ENABLE
    at_lf = ChassisSteeringWheelAngleToMotorAngle(&steering_home[CHASSIS_STEERING_LF], at_lf);
#if CHASSIS_LB_SWERVE_TEST_ENABLE
    at_lb = ChassisSteeringWheelAngleToMotorAngle(&steering_home[CHASSIS_STEERING_LB], at_lb);
#endif
    at_rb = ChassisSteeringWheelAngleToMotorAngle(&steering_home[CHASSIS_STEERING_RB], at_rb);
#endif

    chassis_debug.wheel_speed_lf = vt_lf;
    chassis_debug.wheel_speed_rf = vt_rf;
    chassis_debug.wheel_speed_lb = vt_lb;
    chassis_debug.wheel_speed_rb = vt_rb;
    chassis_debug.steer_ref_lf = at_lf;
    chassis_debug.steer_ref_rf = at_rf;
    chassis_debug.steer_ref_lb = at_lb;
    chassis_debug.steer_ref_rb = at_rb;

#if CHASSIS_SINGLE_SWERVE_TEST_ENABLE
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_LF]);
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_LB]);
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_RB]);
    DJIMotorEnable(motor_steering_rf);
    DJIMotorSetRef(motor_steering_rf, at_rf);
#if CHASSIS_DRIVE_MOTOR_TEST_ENABLE
    if (((CHASSIS_IDLE_DRIVE_HOLD_ENABLE != 0U) && (stop_align_ready != 0U)) ||
        (drive_wait_steering_ready == 0U)) {
        ChassisHoldDriveMotors();
    } else {
        ChassisSetDriveMotorRef(motor_lf, 0.0f, CHASSIS_ID3_M3508_SPEED_DEADBAND);
        ChassisSetDriveMotorRef(motor_rf, vt_rf, CHASSIS_ID1_M3508_SPEED_DEADBAND);
        ChassisSetDriveMotorRef(motor_lb, 0.0f, CHASSIS_ID4_M3508_SPEED_DEADBAND);
        ChassisSetDriveMotorRef(motor_rb, 0.0f, CHASSIS_ID2_M3508_SPEED_DEADBAND);
    }
#else
    ChassisStopDriveMotors();
#endif
#else
    DJIMotorEnable(motor_steering_lf);
    DJIMotorEnable(motor_steering_rf);
#if CHASSIS_LB_SWERVE_TEST_ENABLE
    DJIMotorEnable(motor_steering_lb);
#else
    ChassisSteeringStopMotor(&steering_home[CHASSIS_STEERING_LB]);
#endif
    DJIMotorEnable(motor_steering_rb);
    DJIMotorSetRef(motor_steering_lf, at_lf);
    DJIMotorSetRef(motor_steering_rf, at_rf);
#if CHASSIS_LB_SWERVE_TEST_ENABLE
    DJIMotorSetRef(motor_steering_lb, at_lb);
#endif
    DJIMotorSetRef(motor_steering_rb, at_rb);

#if CHASSIS_DRIVE_MOTOR_TEST_ENABLE
    if (((CHASSIS_IDLE_DRIVE_HOLD_ENABLE != 0U) && (stop_align_ready != 0U)) ||
        (drive_wait_steering_ready == 0U)) {
        ChassisHoldDriveMotors();
    } else {
        ChassisSetDriveMotorRef(motor_lf, vt_lf, CHASSIS_ID3_M3508_SPEED_DEADBAND);
        ChassisSetDriveMotorRef(motor_rf, vt_rf, CHASSIS_ID1_M3508_SPEED_DEADBAND);
#if CHASSIS_LB_SWERVE_TEST_ENABLE
        ChassisSetDriveMotorRef(motor_lb, vt_lb, CHASSIS_ID4_M3508_SPEED_DEADBAND);
#else
        ChassisSetDriveMotorRef(motor_lb, 0.0f, CHASSIS_ID4_M3508_SPEED_DEADBAND);
#endif
        ChassisSetDriveMotorRef(motor_rb, vt_rb, CHASSIS_ID2_M3508_SPEED_DEADBAND);
    }
#else
    ChassisStopDriveMotors();
#endif
#endif
}
void ChassisTask(void)
{
    float vx = 0.0f, vy = 0.0f, vw = 0.0f;
    float raw_vx = 0.0f, raw_vy = 0.0f, raw_vw = 0.0f;
    float raw_translation_norm = 0.0f;
    float linear_deadband = CHASSIS_REMOTE_LINEAR_DEADBAND;
    float angular_deadband = CHASSIS_REMOTE_ANGULAR_DEADBAND;
    float linear_scale = 1.0f;
    float angular_scale = 1.0f;
    float translation_drive_start = CHASSIS_REMOTE_TRANSLATION_DRIVE_START;
    uint8_t fine_control_mode = 0U;
    static uint32_t imu_update_cnt = 0U;
    float imu_dt_s = DWT_GetDeltaT(&imu_update_cnt);
    uint8_t remote_online;

    chassis_debug.task_count++;

    if ((imu_dt_s <= 0.0f) || (imu_dt_s > CHASSIS_IMU_DT_MAX_S)) {
        imu_dt_s = CHASSIS_IMU_DT_FALLBACK_S;
    }
    ChassisIMU_Update(imu_dt_s);
    chassis_debug.imu_online = chassis_imu_data.online;
    chassis_debug.imu_enable = chassis_ctrl_cmd.imu_enable;
    chassis_debug.imu_mode = (uint8_t)chassis_ctrl_cmd.correct_mode;
    chassis_debug.imu_yaw = chassis_imu_data.Yaw;
    chassis_debug.imu_offset_w = chassis_ctrl_cmd.offset_w;
    ChassisSteeringHomeTaskAll();
    ChassisDebugUpdateHomeState();

    remote_online = ChassisRemoteIsOnline();
    chassis_debug.remote_online = remote_online;

    if (remote_online == 0U) {
        // 遥控器掉线时禁止继续使用旧数据，行走电机参考值清零；归零结束后舵向电机也清零停止。
        chassis_debug.stop_reason = 1U;
        chassis_debug.cmd_vx = 0.0f;
        chassis_debug.cmd_vy = 0.0f;
        chassis_debug.cmd_vw = 0.0f;
        chassis_debug.cmd_vw_corrected = 0.0f;
        chassis_debug.imu_offset_w = 0.0f;
        chassis_debug.manual_rotate_active = 0U;
        chassis_debug.rotate_release_settle_active = 0U;
        chassis_debug.fine_control_mode = 0U;
        chassis_debug.translation_dir_change_active = 0U;
        chassis_debug.translation_dir_deg = 0.0f;
        chassis_debug.translation_dir_delta_deg = 0.0f;
        chassis_debug.translation_dir_change_scale = 1.0f;
        chassis_ctrl_cmd.offset_w = 0.0f;
        ChassisIMU_ClearCorrectionPID();
        ChassisStopDriveMotors();
        if (steering_home_all_done != 0U) {
            ChassisStopSteeringMotors();
        }
        return;
    }

    if (remote_data != NULL) {
        raw_vx = (float)remote_boxer.right_y;
        raw_vy = (float)remote_boxer.right_x;
        raw_vw = (float)remote_boxer.left_x;

#if CHASSIS_FINE_CONTROL_ENABLE
        fine_control_mode = (remote_boxer.sb == CHASSIS_FINE_CONTROL_SB_POS) ? 1U : 0U;
#endif
        if (fine_control_mode != 0U) {
            linear_deadband = CHASSIS_FINE_LINEAR_DEADBAND;
            angular_deadband = CHASSIS_FINE_ANGULAR_DEADBAND;
            linear_scale = CHASSIS_FINE_LINEAR_SCALE;
            angular_scale = CHASSIS_FINE_ANGULAR_SCALE;
            translation_drive_start = CHASSIS_FINE_TRANSLATION_DRIVE_START;
        }

        if (fabsf(raw_vx) < linear_deadband) raw_vx = 0.0f;
        if (fabsf(raw_vy) < linear_deadband) raw_vy = 0.0f;
        if (fabsf(raw_vw) < angular_deadband) raw_vw = 0.0f;

        raw_translation_norm = sqrtf(raw_vx * raw_vx + raw_vy * raw_vy);
        if (raw_translation_norm > translation_drive_start) {
            chassis_translation_drive_scale =
                (raw_translation_norm - translation_drive_start) /
                raw_translation_norm;
        } else if (raw_vw != 0.0f) {
            chassis_translation_drive_scale = 1.0f;
        } else {
            chassis_translation_drive_scale = 0.0f;
        }

        // 右摇杆 Y 轴 -> 前后线速度 vx
        vx = CHASSIS_REMOTE_VX_SIGN * raw_vx / REMOTE_STICK_RANGE * REMOTE_MAX_LINEAR * linear_scale;
        // 右摇杆 X 轴 -> 左右线速度 vy
        vy = CHASSIS_REMOTE_VY_SIGN * raw_vy / REMOTE_STICK_RANGE * REMOTE_MAX_LINEAR * linear_scale;
        // 左摇杆 X 轴 -> 旋转角速度 vw
        vw = CHASSIS_REMOTE_VW_SIGN * raw_vw / REMOTE_STICK_RANGE * REMOTE_MAX_ANGULAR * angular_scale;

#if CHASSIS_TRANSLATION_DIR_CHANGE_ENABLE
        chassis_translation_drive_scale *=
            ChassisTranslationDirectionChangeScale(vx, vy, raw_translation_norm, translation_drive_start);
#else
        chassis_debug.translation_dir_change_active = 0U;
        chassis_debug.translation_dir_delta_deg = 0.0f;
        chassis_debug.translation_dir_change_scale = 1.0f;
#endif

        chassis_debug.fine_control_mode = fine_control_mode;
        chassis_debug.remote_right_y = (float)remote_boxer.right_y;
        chassis_debug.remote_right_x = (float)remote_boxer.right_x;
        chassis_debug.remote_left_x = (float)remote_boxer.left_x;

        if (ChassisRemoteYawPresetTask(remote_boxer.sc) != 0U) {
            vw = 0.0f;
        }
    }

    chassis_debug.cmd_vx = vx;
    chassis_debug.cmd_vy = vy;
    chassis_debug.cmd_vw = vw;

    SteeringWheelKinematics(vx, vy, vw);
		
		
//	DJIMotorSetRef(motor_steering_rf,STEERING_CHASSIS_ALIGN_ANGLE_1);
//	DJIMotorSetRef(motor_steering_lf,STEERING_CHASSIS_ALIGN_ANGLE_3);
//	DJIMotorSetRef(motor_steering_rb,STEERING_CHASSIS_ALIGN_ANGLE_2);
}
