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
    float Pitch;             // 俯仰角 (Degree)
    float Roll;              // 横滚角 (Degree)
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

/**
 * @brief 底盘调试镜像变量，便于Keil Watch直接观察static/局部状态
 */
typedef struct
{
    uint32_t task_count;             // ChassisTask执行次数
    uint8_t remote_online;           // 遥控器在线状态
    uint8_t steering_home_all_done;  // 舵轮归零总完成标志
    uint8_t stop_reason;             // 0=正常,1=遥控掉线,2=归零未完成
    uint8_t home_state[4];           // 四个舵轮归零状态
    uint8_t home_done[4];            // 四个舵轮单独归零完成标志
    float cmd_vx;                    // 当前前后速度指令
    float cmd_vy;                    // 当前左右速度指令
    float cmd_vw;                    // 当前旋转速度指令
    float cmd_vw_corrected;          // 叠加 IMU 修正后的旋转速度指令
    uint8_t imu_online;              // IMU 在线状态
    uint8_t imu_enable;              // IMU 修正使能状态
    uint8_t imu_mode;                // IMU 修正模式
    float imu_yaw;                   // 当前 IMU 航向角
    float imu_offset_w;              // 当前 IMU 航向修正量
    float remote_right_y;            // 右摇杆Y原始中心化值
    float remote_right_x;            // 右摇杆X原始中心化值
    float remote_left_x;             // 左摇杆X原始中心化值
    float wheel_speed_lf;            // 左前行走轮目标速度
    float wheel_speed_rf;            // 右前行走轮目标速度
    float wheel_speed_lb;            // 左后行走轮目标速度
    float wheel_speed_rb;            // 右后行走轮目标速度
    float steer_ref_lf;              // 左前舵向目标
    float steer_ref_rf;              // 右前舵向目标
    float steer_ref_lb;              // 左后舵向目标
    float steer_ref_rb;              // 右后舵向目标
} ChassisDebug_s;

extern ChassisCtrlCmd_s chassis_ctrl_cmd;
extern ChassisDebug_s chassis_debug;

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
