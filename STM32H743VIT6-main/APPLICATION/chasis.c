/**
 * @file chassis.c
 * @author 
 * @brief 底盘控制实现，包含电机初始化、IMU 处理和运动学分解
 * @version 0.1
 * @date 2024-xx-xx
 */

#include "chassis.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "robot_def.h"
#include <math.h>

#define CHASSIS_ID1_M3508_SPEED_DEADBAND 300.0f
#define CHASSIS_STEERING_STARTUP_HOLD_TICKS 50U
#define CHASSIS_STEERING_ANGLE_MAX_OUT 1200.0f
#define CHASSIS_STEERING_SPEED_MAX_OUT 6000.0f
#define CHASSIS_GM6020_ID3_OUTPUT_REVERSE 1U

/* 底盘行走电机实例（3508 电机）: 左前, 右前, 左后, 右后 */
static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb;

/* 底盘转向电机实例（6020 电机）: 左前, 右前, 左后, 右后 */
static DJIMotor_Instance *motor_steering_lf, *motor_steering_rf, *motor_steering_lb, *motor_steering_rb;

/* 航向锁定 PID 控制器 */
static PID_Instance chassis_follow_pid;

/* 临时目标轮速与角度（用于某些特殊运动模型） */
static float vt_lf, vt_rf, vt_lb, vt_rb;
static float at_lf, at_rf, at_lb, at_rb;

/* 底盘 IMU 内部数据存储 */
static ChassisIMUData_s chassis_imu_data;

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

/**
 * @brief 底盘 IMU 数据及其控制结构体初始化
 */
static void ChassisIMU_Init(void)
{
    chassis_imu_data.Yaw = 0.0f;
    chassis_imu_data.GyroZ = 0.0f;
    chassis_imu_data.status = g_bmi088_status;
    chassis_imu_data.online = (g_bmi088_status == BMI088_OK) ? 1U : 0U;

    chassis_ctrl_cmd.Chassis_IMU_data = &chassis_imu_data;
    chassis_ctrl_cmd.imu_enable = chassis_imu_data.online;
    chassis_ctrl_cmd.correct_mode = IMU_CORRECT_STRAIGHT;
    chassis_ctrl_cmd.last_yaw = chassis_imu_data.Yaw;
    chassis_ctrl_cmd.target_yaw = chassis_imu_data.Yaw;
    chassis_ctrl_cmd.offset_w = 0.0f;
}

/**
 * @brief 更新底盘 IMU 航向信息，通过 Z 轴角速度积分实现
 * @param dt_s 采样周期 (秒)
 */
static void ChassisIMU_Update(float dt_s)
{
    BMI088_Status_t status;

    if (dt_s <= 0.0f) {
        return;
    }

    status = BMI088_ReadAll(&g_bmi088_data);
    chassis_imu_data.status = status;

    if (status != BMI088_OK) {
        chassis_imu_data.online = 0U;
        chassis_ctrl_cmd.imu_enable = 0U;
        return;
    }

    chassis_imu_data.online = 1U;
    chassis_imu_data.GyroZ = g_bmi088_data.gyro_rads.z;
    chassis_imu_data.Yaw = ChassisIMU_NormalizeDeg(chassis_imu_data.Yaw + chassis_imu_data.GyroZ * dt_s * RAD_2_DEGREE);
}

void ChassisIMU_Enable(uint8_t enable)
{
    chassis_ctrl_cmd.imu_enable = (enable != 0U) && (chassis_imu_data.online != 0U);
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
    chassis_imu_data.Yaw = ChassisIMU_NormalizeDeg(yaw_deg);
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
    static uint8_t hold_ticks = 0U;

    if (motor_steering_lb == NULL || motor_steering_lb->feedback_initialized == 0U) {
        hold_ticks = 0U;
        if (motor_steering_lb != NULL) {
            DJIMotorStop(motor_steering_lb);
        }
        return 0U;
    }

    if (hold_ticks < CHASSIS_STEERING_STARTUP_HOLD_TICKS) {
        hold_ticks++;
        DJIMotorSetRef(motor_steering_lb, motor_steering_lb->measure.total_angle);
        DJIMotorEnable(motor_steering_lb);
        return 0U;
    }

    return 1U;
}

void ChassisInit()
{
    // 四个行走电机参数基本一致，主要区别是 CAN ID 和电机方向。
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.fdcan_handle   = &hfdcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 4, // 3
                .Ki            = 0.2, // 0.5
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

    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
       motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    chassis_motor_config.controller_param_init_config.speed_PID.DeadBand   = CHASSIS_ID1_M3508_SPEED_DEADBAND;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);
    chassis_motor_config.controller_param_init_config.speed_PID.DeadBand   = 0.0f;

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
            chassis_motor_config.controller_param_init_config.speed_PID.Kp         =2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);


    // 6020 转向电机初始化。
    Motor_Init_Config_s chassis_motor_steering_config = {
        .can_init_config.fdcan_handle   = &hfdcan2,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 8,
                .Ki                = 0.2,
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
                .Kp            = 40,
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
    chassis_motor_steering_config.can_init_config.tx_id = 4;
    motor_steering_lf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 1;
    motor_steering_rf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 3;
#if CHASSIS_GM6020_ID3_OUTPUT_REVERSE
    chassis_motor_steering_config.controller_setting_init_config.output_reverse_flag = MOTOR_DIRECTION_REVERSE;
#endif
    motor_steering_lb                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.controller_setting_init_config.output_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_steering_config.can_init_config.tx_id = 2;
    motor_steering_rb                                   = DJIMotorInit(&chassis_motor_steering_config);
    if (motor_steering_lb != NULL) {
        DJIMotorStop(motor_steering_lb);
    }

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

/**
 * @brief 舵轮底盘运动学解算。
 * @param vx 前后方向线速度指令。
 * @param vy 左右方向线速度指令。
 * @param vw 角速度指令。
 * @note 当前使用三轮舵轮模式：3 号轮为左前轮，1 号轮为左后轮，2 号轮为右后轮，4 号轮为右前轮。
 */
void SteeringWheelKinematics(float vx, float vy, float vw)
{
    float chassis_vx = vx;
    float chassis_vy = vy;
    float chassis_vw = vw;
    static uint8_t first_run_kinematics = 1;
    float offset_lf = 0.0f, offset_rf = 0.0f, offset_lb = 0.0f, offset_rb = 0.0f;
    float at_lf_last = 0.0f, at_rf_last = 0.0f, at_lb_last = 0.0f, at_rb_last = 0.0f;
    uint8_t id3_ready = 0U;

    id3_ready = ChassisSteeringId3StartupGuard();

    at_lf_last = ChassisSteeringAngle(motor_steering_lf);
    at_rf_last = ChassisSteeringAngle(motor_steering_rf);
    at_lb_last = ChassisSteeringAngle(motor_steering_lb);
    at_rb_last = ChassisSteeringAngle(motor_steering_rb);

    if (first_run_kinematics) {
        chassis_ctrl_cmd.last_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;
        first_run_kinematics = 0;
    }

    chassis_ctrl_cmd.offset_w = UpdateIMUCorrection(vw);
    chassis_vw = vw + chassis_ctrl_cmd.offset_w;

    {
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
    }

    at_lf = STEERING_CHASSIS_ALIGN_ANGLE_LF + offset_lf;
    at_rf = STEERING_CHASSIS_ALIGN_ANGLE_RF + offset_rf;
    at_lb = STEERING_CHASSIS_ALIGN_ANGLE_LB + offset_lb;
    at_rb = STEERING_CHASSIS_ALIGN_ANGLE_RB + offset_rb;

    ANGLE_LIMIT_360_TO_180_ABS(at_lf);
    ANGLE_LIMIT_360_TO_180_ABS(at_rf);
    ANGLE_LIMIT_360_TO_180_ABS(at_lb);
    ANGLE_LIMIT_360_TO_180_ABS(at_rb);

    MinmizeRotation(&at_lf, &at_lf_last, &vt_lf);
    MinmizeRotation(&at_rf, &at_rf_last, &vt_rf);
    MinmizeRotation(&at_lb, &at_lb_last, &vt_lb);
    MinmizeRotation(&at_rb, &at_rb_last, &vt_rb);

    DJIMotorSetRef(motor_steering_lf, at_lf);
    DJIMotorSetRef(motor_steering_rf, at_rf);
    if (id3_ready) {
        DJIMotorEnable(motor_steering_lb);
        DJIMotorSetRef(motor_steering_lb, at_lb);
    } else {
        vt_lb = 0.0f;
    }
    DJIMotorSetRef(motor_steering_rb, at_rb);

    if (chassis_vw == 0.0f && chassis_vx == 0.0f && chassis_vy == 0.0f) {
        DJIMotorSetRef(motor_lf, 0);
        DJIMotorSetRef(motor_rf, 0);
        DJIMotorStop(motor_rf);
        DJIMotorSetRef(motor_lb, 0);
        DJIMotorSetRef(motor_rb, 0);
    } else {
        DJIMotorSetRef(motor_lf, vt_lf);

        if (fabsf(vt_rf) < CHASSIS_ID1_M3508_SPEED_DEADBAND) {
            vt_rf = 0.0f;
            DJIMotorSetRef(motor_rf, 0);
            DJIMotorStop(motor_rf);
        } else {
            DJIMotorEnable(motor_rf);
            DJIMotorSetRef(motor_rf, vt_rf);
        }

        DJIMotorSetRef(motor_lb, vt_lb);
        DJIMotorSetRef(motor_rb, vt_rb);
    }
}
void ChassisTask(void)
{
    float vx = 0.0f, vy = 0.0f, vw = 0.0f;

    ChassisIMU_Update(0.001f);

    if (remote_data != NULL) {
        // 左摇杆 Y轴 → 前后线速度 vx
        vx = (float)remote_data->rocker_l1 / REMOTE_STICK_RANGE * REMOTE_MAX_LINEAR;
        // 左摇杆 X轴 → 左右线速度 vy
        // 这里反相，修正底盘左右方向与摇杆输入相反的问题
        vy = -(float)remote_data->rocker_l_ / REMOTE_STICK_RANGE * REMOTE_MAX_LINEAR;
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
