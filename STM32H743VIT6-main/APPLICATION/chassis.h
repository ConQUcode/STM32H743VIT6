/**
 * @file chassis.h
 * @author 
 * @brief 底盘控制逻辑头文件，包含底盘状态结构体定义和控制函数声明
 * @version 0.1
 * @date 2024-xx-xx
 */

#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#include "BMI088.h"
#include <stdint.h>

/**
 * @brief 底盘 IMU 修正模式枚举
 */
typedef enum
{
    IMU_CORRECT_STRAIGHT = 0, // 直行航向锁定模式
    IMU_CORRECT_ROTATION,    // 旋转航向控制模式
    IMU_CORRECT_HYBRID       // 混合控制模式
} ChassisIMUCorrectMode_e;

/**
 * @brief 底盘 IMU 数据结构体
 */
typedef struct
{
    float Yaw;               // 航向角 (Degree)
    float GyroZ;             // 角速度 (rad/s)
    uint8_t online;          // 传感器在线状态
    BMI088_Status_t status;  // 传感器硬件状态
} ChassisIMUData_s;

/**
 * @brief 底盘控制命令结构体
 */
typedef struct
{
    uint8_t imu_enable;                 // IMU 修正使能开关
    ChassisIMUData_s *Chassis_IMU_data; // 指向 IMU 数据结构体
    ChassisIMUCorrectMode_e correct_mode; // 当前修正模式
    float last_yaw;                     // 上一次锁定的航向角
    float target_yaw;                   // 目标航向角
    float offset_w;                     // 航向修正计算出的角速度增量
} ChassisCtrlCmd_s;

extern ChassisCtrlCmd_s chassis_ctrl_cmd;

/**
 * @brief 底盘初始化，配置行走电机和转向电机的 PID 参数及 CAN 通信
 */
void ChassisInit(void);

/**
 * @brief 底盘控制任务，处理遥控器数据、运动学分解和电机控制
 */
void ChassisTask(void);

/**
 * @brief 底盘校准函数
 */
void Jiaozhun(void);

/**
 * @brief 舵轮底盘运动学正分解
 * @param vx 前后线速度 (mm/s 或 normalized)
 * @param vy 左右线速度 (mm/s 或 normalized)
 * @param vw 旋转角速度 (deg/s 或 normalized)
 */
void SteeringWheelKinematics(float vx, float vy, float vw);

/**
 * @brief 旧版运动学计算函数
 */
void SteeringWheelKinematics_old(float vx, float vy, float vw);

/**
 * @brief 旧版底盘测试函数
 */
void ChassisTest_OldVersion();

/**
 * @brief 使能或禁用 IMU 航向修正
 * @param enable 1 为使能, 0 为禁用
 */
void ChassisIMU_Enable(uint8_t enable);

/**
 * @brief 设置 IMU 航向修正模式
 * @param mode 见 ChassisIMUCorrectMode_e
 */
void ChassisIMU_SetCorrectMode(ChassisIMUCorrectMode_e mode);

/**
 * @brief 重置当前航向角偏移量
 * @param yaw_deg 起始航向角
 */
void ChassisIMU_ResetYaw(float yaw_deg);

extern  float V;

#endif
