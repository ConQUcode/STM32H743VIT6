#include "chassis.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "robot_def.h"
#include <math.h>


static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb;
static DJIMotor_Instance *motor_steering_lf, *motor_steering_rf, *motor_steering_lb, *motor_steering_rb;
static PID_Instance chassis_follow_pid;
static float vt_lf, vt_rf, vt_lb, vt_rb;
static float at_lf, at_rf, at_lb, at_rb;
static ChassisIMUData_s chassis_imu_data;

ChassisCtrlCmd_s chassis_ctrl_cmd = {
    .imu_enable = 0U,
    .Chassis_IMU_data = &chassis_imu_data,
    .correct_mode = IMU_CORRECT_STRAIGHT,
    .last_yaw = 0.0f,
    .target_yaw = 0.0f,
    .offset_w = 0.0f,
};

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

static float ChassisIMU_DiffDeg(float target_deg, float current_deg)
{
    return ChassisIMU_NormalizeDeg(target_deg - current_deg);
}

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
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
       motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
            chassis_motor_config.controller_param_init_config.speed_PID.Kp         =2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

             Motor_Init_Config_s chassis_motor_config1 = {
        .can_init_config.fdcan_handle   = &hfdcan1,
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

    // 6020 转向电机初始化。
    Motor_Init_Config_s chassis_motor_steering_config = {
        .can_init_config.fdcan_handle   = &hfdcan2,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 12,
                .Ki                = 0.2,
                .Kd                = 0,
                .CoefA             = 5,
                .CoefB             = 0.1,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .IntegralLimit     = 1000,
                .MaxOut            = 16000,
                .Derivative_LPF_RC = 0.001,
                .DeadBand          = 0.5,
            },
            .speed_PID = {
                .Kp            = 40,
                .Ki            = 3,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
                .IntegralLimit = 4000,
                .MaxOut        = 20000,
                .Output_LPF_RC = 0.03,
            },

        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    chassis_motor_steering_config.can_init_config.tx_id = 4;
    motor_steering_lf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 1;
    motor_steering_rf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 3;
    motor_steering_lb                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 2;
    motor_steering_rb                                   = DJIMotorInit(&chassis_motor_steering_config);

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
    float rotation = *angle - *last_angle;

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
 * @note 当前使用三轮舵轮模式：3 号轮为前轮，1 号轮为左后轮，2 号轮为右后轮。
 */
void SteeringWheelKinematics(float vx, float vy, float vw)
{
    float offset_1, offset_2, offset_3;
    float at_1_last, at_2_last, at_3_last;
    float vt_1, vt_2, vt_3;
    float at_1, at_2, at_3;

    /*
    float offset_lf, offset_rf, offset_lb, offset_rb;
    float at_lf_last, at_rf_last, at_lb_last, at_rb_last;
    */

    float chassis_vx = 0;
    float chassis_vy = 0;
    float chassis_vw = 0;
    static uint8_t first_run_kinematics = 1;

    // 当前三轮模式的指针映射：lf->3 号前轮，lb->1 号左后轮，rb->2 号右后轮。
    at_3_last = motor_steering_lf->measure.total_angle;
    at_1_last = motor_steering_lb->measure.total_angle;
    at_2_last = motor_steering_rb->measure.total_angle;

    /*
    at_lb_last = motor_steering_lb->measure.total_angle;
    at_lf_last = motor_steering_lf->measure.total_angle;
    at_rf_last = motor_steering_rf->measure.total_angle;
    at_rb_last = motor_steering_rb->measure.total_angle;
    */

    if(first_run_kinematics) {
        chassis_ctrl_cmd.last_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;
        first_run_kinematics = 0;
    }

    chassis_ctrl_cmd.offset_w = UpdateIMUCorrection(vw);
    chassis_vw = vw + chassis_ctrl_cmd.offset_w;

    chassis_vx = vx;
    chassis_vy = vy;

    float w = chassis_vw;

    // 三轮舵轮运动学解算。
    float temp_x_3 = chassis_vx - w * W3_Y;
    float temp_y_3 = chassis_vy + w * W3_X;

    float temp_x_1 = chassis_vx - w * W1_Y;
    float temp_y_1 = chassis_vy + w * W1_X;

    float temp_x_2 = chassis_vx - w * W2_Y;
    float temp_y_2 = chassis_vy + w * W2_X;

    vt_1 = sqrtf(temp_x_1 * temp_x_1 + temp_y_1 * temp_y_1);
    vt_2 = sqrtf(temp_x_2 * temp_x_2 + temp_y_2 * temp_y_2);
    vt_3 = sqrtf(temp_x_3 * temp_x_3 + temp_y_3 * temp_y_3);

    offset_1 = atan2f(temp_y_1, temp_x_1) * RAD_2_DEGREE;
    offset_2 = atan2f(temp_y_2, temp_x_2) * RAD_2_DEGREE;
    offset_3 = atan2f(temp_y_3, temp_x_3) * RAD_2_DEGREE;

    /* 四轮模式解算保留，后续需要切换时可恢复。
    float temp_x = chassis_vx - w, temp_y = chassis_vy - w;
    arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_lf);
    temp_y = chassis_vy + w;
    arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_lb);
    temp_x = chassis_vx + w;
    arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_rb);
    temp_y = chassis_vy - w;
    arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_rf);

    offset_lf = atan2f(chassis_vy - w, chassis_vx - w) * RAD_2_DEGREE;
    offset_rf = atan2f(chassis_vy - w, chassis_vx + w) * RAD_2_DEGREE;
    offset_lb = atan2f(chassis_vy + w, chassis_vx - w) * RAD_2_DEGREE;
    offset_rb = atan2f(chassis_vy + w, chassis_vx + w) * RAD_2_DEGREE;
    */

    at_3 = STEERING_CHASSIS_ALIGN_ANGLE_3 + offset_3;
    at_1 = STEERING_CHASSIS_ALIGN_ANGLE_1 + offset_1;
    at_2 = STEERING_CHASSIS_ALIGN_ANGLE_2 + offset_2;

    ANGLE_LIMIT_360_TO_180_ABS(at_3);
    ANGLE_LIMIT_360_TO_180_ABS(at_1);
    ANGLE_LIMIT_360_TO_180_ABS(at_2);

    MinmizeRotation(&at_3, &at_3_last, &vt_3);
    MinmizeRotation(&at_1, &at_1_last, &vt_1);
    MinmizeRotation(&at_2, &at_2_last, &vt_2);

    /* 四轮模式目标角度保留，后续需要切换时可恢复。
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
    */

    DJIMotorSetRef(motor_steering_lf, at_3);
    DJIMotorSetRef(motor_steering_rf, at_1);
    DJIMotorSetRef(motor_steering_rb, at_2);

    if(w == 0 && vx == 0 && vy == 0) {
        DJIMotorSetRef(motor_lf, 0);
        DJIMotorSetRef(motor_lb, 0);
        DJIMotorSetRef(motor_rb, 0);
    } else {
        DJIMotorSetRef(motor_lf, vt_3);
        DJIMotorSetRef(motor_rf, vt_1);
        DJIMotorSetRef(motor_rb, vt_2);
    }

    /* 四轮模式下发目标保留，后续需要切换时可恢复。
    DJIMotorSetRef(motor_steering_lf, at_lf);
    DJIMotorSetRef(motor_steering_rf, at_rf);
    DJIMotorSetRef(motor_steering_lb, at_lb);
    DJIMotorSetRef(motor_steering_rb, at_rb);

    if(w == 0 && vx == 0 && vy == 0) {
        DJIMotorSetRef(motor_lf, 0);
        DJIMotorSetRef(motor_rf, 0);
        DJIMotorSetRef(motor_lb, 0);
        DJIMotorSetRef(motor_rb, 0);
    } else {
        DJIMotorSetRef(motor_lf, vt_lf);
        DJIMotorSetRef(motor_rf, vt_rf);
        DJIMotorSetRef(motor_lb, vt_lb);
        DJIMotorSetRef(motor_rb, vt_rb);
    }
    */
}

void ChassisTask(void)
{
    ChassisIMU_Update(0.001f);
    SteeringWheelKinematics(0.0f, 0.0f, 0.0f);
}
