#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <stdint.h>

/* ===================== 机械臂结构参数 (mm) ===================== */
#define ARM_L1 360.0f  // 大臂长度 (DM4310 → M3508)
#define ARM_L2 290.0f  // 小臂长度 (M3508 → 舵机)
#define ARM_L3 198.0f  // 舵机到末端距离

/* ===================== 关节限位 (度) ===================== */
/* 根据实际机械结构修改 */
#define ARM_J1_MIN_DEG -180.0f
#define ARM_J1_MAX_DEG 180.0f
#define ARM_J2_MIN_DEG -180.0f
#define ARM_J2_MAX_DEG 180.0f
#define ARM_J3_MIN_DEG -180.0f
#define ARM_J3_MAX_DEG 180.0f

/* ===================== 类型定义 ===================== */

typedef struct {
    float x;   // 末端 X 坐标 (mm)
    float y;   // 末端 Y 坐标 (mm)
    float phi; // 末端姿态角 (度, 相对于 X 轴正方向)
} Arm_Position_t;

typedef struct {
    float j1;  // 大臂角度 (度, DM4310, 绝对角度)
    float j2;  // 小臂角度 (度, M3508, 相对于大臂)
    float j3;  // 舵机角度 (度, 相对于小臂)
} Arm_JointAngles_t;

typedef enum {
    ARM_ELBOW_DOWN = 0, // 肘向下 (θ2 > 0, 自然姿态)
    ARM_ELBOW_UP   = 1, // 肘向上 (θ2 < 0)
} Arm_Elbow_Config_e;

/* ===================== 公有 API ===================== */

/**
 * @brief 正运动学: 关节角度 → 末端位姿
 * @param angles  关节角度 (度)
 * @return 末端位姿 (x, y, phi)
 */
Arm_Position_t Arm_FK(Arm_JointAngles_t angles);

/**
 * @brief 逆运动学: 末端位置 → 关节角度 (仅 J1/J2, 忽略 J3)
 * @param target      目标末端位姿
 * @param angles_out  输出关节角度
 * @param elbow       肘部构型选择
 * @return 1=解算成功, 0=目标不可达
 */
uint8_t Arm_IK(Arm_Position_t target, Arm_JointAngles_t *angles_out,
               Arm_Elbow_Config_e elbow);

/**
 * @brief 检查关节角度是否在限位内
 * @param angles  关节角度
 * @return 1=合法, 0=超限
 */
uint8_t Arm_CheckJointLimits(Arm_JointAngles_t angles);

/**
 * @brief 将角度归一化到 [-180, 180) 范围
 * @param angle_deg  任意角度 (度)
 * @return 归一化后的角度
 */
float Arm_NormalizeAngle(float angle_deg);

#endif
