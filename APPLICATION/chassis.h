#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#include "BMI088.h"
#include <stdint.h>

typedef enum
{
    IMU_CORRECT_STRAIGHT = 0,
    IMU_CORRECT_ROTATION,
    IMU_CORRECT_HYBRID
} ChassisIMUCorrectMode_e;

typedef struct
{
    float Yaw;
    float GyroZ;
    uint8_t online;
    BMI088_Status_t status;
} ChassisIMUData_s;

typedef struct
{
    uint8_t imu_enable;
    ChassisIMUData_s *Chassis_IMU_data;
    ChassisIMUCorrectMode_e correct_mode;
    float last_yaw;
    float target_yaw;
    float offset_w;
} ChassisCtrlCmd_s;

extern ChassisCtrlCmd_s chassis_ctrl_cmd;

void ChassisInit(void);
void ChassisTask(void);
void Jiaozhun(void);
void SteeringWheelKinematics(float vx, float vy, float vw);
void SteeringWheelKinematics_old(float vx, float vy, float vw);
void ChassisTest_OldVersion();
void ChassisIMU_Enable(uint8_t enable);
void ChassisIMU_SetCorrectMode(ChassisIMUCorrectMode_e mode);
void ChassisIMU_ResetYaw(float yaw_deg);
extern  float V;

#endif
