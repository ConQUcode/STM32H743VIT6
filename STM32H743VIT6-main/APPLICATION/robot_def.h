/**
 * @file robot_def.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   机器人运动定义, 包含机器人的各种参数
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __ROBOT_DEF_H__
#define __ROBOT_DEF_H__

#include "stdint.h"


#define RAD_2_DEGREE        57.2957795f    // 180/pi
#define ANGLE_LIMIT_360_TO_180_ABS(angle) \
    do { \
        while ((angle) > 180.0f) { (angle) -= 360.0f; } \
        while ((angle) < -180.0f) { (angle) += 360.0f; } \
    } while (0)

// 机器人几何尺寸定义
// 长410mm, 宽320mm. 假设长为前后方向(轴距), 宽为左右方向(轮距)
#define CHASSIS_WHEEL_BASE      0.410f      // 轴距(前后轮中心距, X轴方向)
#define CHASSIS_WHEEL_TRACK     0.320f      // 轮距(左右轮中心距, Y轴方向)
#define CHASSIS_HALF_BASE       (CHASSIS_WHEEL_BASE / 2.0f)
#define CHASSIS_HALF_TRACK      (CHASSIS_WHEEL_TRACK / 2.0f)

// 舵轮方位相对于机器人中心的坐标定义 (单位: 这里的坐标是730/816/816等常用的)
// 3号轮，中间 (正前)
#define W3_X  (0.4865f)
#define W3_Y  (0.0f)   

// 1号轮，底部左顶点 (左后) -> 假设1为左2为右
#define W1_X  (-0.2433f)
#define W1_Y  (-0.3650f)

// 2号轮，底部右顶点 (右后)
#define W2_X  (-0.2433f)
#define W2_Y  (0.3650f)

// 电机速度转换相关宏定义 
#define MOTOR_REDUCTION_RATIO    19.2032f     // 电机减速比 1:19.2302
#define WHEEL_RADIUS_M           0.081f     // 车轮半径(米) - 根据实际车轮尺寸调整

// 线速度转换为3508电机转速指令的转换系数
// 转换公式: 电位RPM = 线速度(m/s) * 转换系数
// 推导过程:
// 1. 车轮转速(rpm) = 线速度(m/s) * 60 / (2 * PI * 车轮半径)
// 2. 电机转速(rpm) = 车轮转速(rpm) * 减速比
// 3. 转换系数 = 60 * 减速比 / (2 * PI * 车轮半径)
#define LINEAR_VELOCITY_TO_MOTOR_RPM    (60.0f * MOTOR_REDUCTION_RATIO / (2.0f * 3.14159f * WHEEL_RADIUS_M)) // 约等于 8732.24

// 将电机转速指令转换回线速度 (便于速度反馈显示等)
#define MOTOR_SPEED_TO_LINEAR_VEL(motor_speed)   ((motor_speed) / LINEAR_VELOCITY_TO_MOTOR_RPM)

// DJ6020拨弹电机速度转换相关宏定义
#define GM6020_REDUCTION_RATIO    1.0f        // GM6020通常为直驱或减速比1:1
#define GM6020_ECD_RANGE          8192.0f     // 编码器范围 0-8191

// 角速度(rad/s)转换为GM6020输出RPM
// 转换公式: 输出RPM = 角速度(rad/s) * 60 / (2 * PI) * 减速比
#define ANGULAR_VELOCITY_TO_GM6020_RPM    (60.0f * GM6020_REDUCTION_RATIO / (2.0f * 3.14159f))

// 将输出轴的GM6020 RPM转换为角速度(rad/s)
#define GM6020_RPM_TO_ANGULAR_VEL(rpm)    ((rpm) / ANGULAR_VELOCITY_TO_GM6020_RPM)


#define STEERING_CHASSIS_ALIGN_ECD_LF   7896// 舵机 A 4号编码值（减小往逆时针）
#define STEERING_CHASSIS_ALIGN_ECD_LB   5796 // 舵机 B 3号编码值
#define STEERING_CHASSIS_ALIGN_ECD_RF   6800// 舵机 C 1号编码值
#define STEERING_CHASSIS_ALIGN_ECD_RB   8142 // 舵机 D 2号编码值

#define STEERING_CHASSIS_ALIGN_ANGLE_LF STEERING_CHASSIS_ALIGN_ECD_LF / 8192.f * 360.f // 舵机 A 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_LB STEERING_CHASSIS_ALIGN_ECD_LB / 8192.f * 360.f // 舵机 B 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_RF STEERING_CHASSIS_ALIGN_ECD_RF / 8192.f * 360.f // 舵机 C 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_RB STEERING_CHASSIS_ALIGN_ECD_RB / 8192.f * 360.f // 舵机 D 对齐角度

// ================== 三舵轮底盘定义 (根据实际硬件值) ==================
// 假设：1号为左后，2号为右后，3号为正前方
#define STEERING_CHASSIS_ALIGN_ECD_1   6800 // 舵轮1号垂直机械零值700
#define STEERING_CHASSIS_ALIGN_ECD_2   8142 // 舵轮2号垂直机械零值8011
#define STEERING_CHASSIS_ALIGN_ECD_3   150 // 舵轮3号垂直机械零值1240

#define STEERING_CHASSIS_ALIGN_ANGLE_1 STEERING_CHASSIS_ALIGN_ECD_1 / 8192.f * 360.f
#define STEERING_CHASSIS_ALIGN_ANGLE_2 STEERING_CHASSIS_ALIGN_ECD_2 / 8192.f * 360.f
#define STEERING_CHASSIS_ALIGN_ANGLE_3 STEERING_CHASSIS_ALIGN_ECD_3 / 8192.f * 360.f

/* 遥控器摇杆映射参数 */
#define REMOTE_STICK_RANGE    660.0f   // 摇杆最大行程值(±660为DJI DT7典型值)
#define REMOTE_MAX_LINEAR     15000.0f  // 最大线速度 (deg/s, 约333 RPM)
#define REMOTE_MAX_ANGULAR    15000.0f  // 最大角速度 (deg/s)
#define REMOTE_DEADBAND       50.0f    // 摇杆死区 (deg/s)

#pragma pack(1) // 压缩结构体,取消字节对齐,使串口数据读取能直接拷贝

#pragma pack() // 取消压缩
#endif