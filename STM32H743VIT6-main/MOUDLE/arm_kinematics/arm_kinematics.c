#include "arm_kinematics.h"
#include "general_def.h"
#include <math.h>
#include <stddef.h>

/* ===================== 正运动学 ===================== */

Arm_Position_t Arm_FK(Arm_JointAngles_t angles)
{
    Arm_Position_t pos;

    float a1  = angles.j1 * DEGREE_2_RAD;
    float a2  = angles.j2 * DEGREE_2_RAD;
    float a3  = angles.j3 * DEGREE_2_RAD;
    float a12 = a1 + a2;
    float a123 = a12 + a3;

    pos.x   = ARM_L1 * cosf(a1) + ARM_L2 * cosf(a12) + ARM_L3 * cosf(a123);
    pos.y   = ARM_L1 * sinf(a1) + ARM_L2 * sinf(a12) + ARM_L3 * sinf(a123);
    pos.phi = Arm_NormalizeAngle(angles.j1 + angles.j2 + angles.j3);

    return pos;
}

/* ===================== 逆运动学 ===================== */

uint8_t Arm_IK(Arm_Position_t target, Arm_JointAngles_t *angles_out,
               Arm_Elbow_Config_e elbow)
{
    if (angles_out == NULL)
        return 0;

    /* 2 连杆逆解: 只解算 J1/J2, 输入 target.x/y 直接当作腕点 (L2 末端) 坐标,
     * phi 和 J3 不参与解算, J3 由调用方自行处理 */
    float x_w = target.x;
    float y_w = target.y;

    float r2 = x_w * x_w + y_w * y_w;
    float r  = sqrtf(r2);

    if (r > ARM_L1 + ARM_L2 + 0.01f || r < fabsf(ARM_L1 - ARM_L2) - 0.01f)
        return 0;  /* 目标超出 2 连杆工作空间 */

    float cos_j2 = (r2 - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2) /
                   (2.0f * ARM_L1 * ARM_L2);
    if (cos_j2 > 1.0f)  cos_j2 = 1.0f;
    if (cos_j2 < -1.0f) cos_j2 = -1.0f;

    float j2_rad;
    if (elbow == ARM_ELBOW_DOWN)
        j2_rad = acosf(cos_j2);      /* θ2 > 0, 正解 */
    else
        j2_rad = -acosf(cos_j2);     /* θ2 < 0, 负解 */

    float j1_rad = atan2f(y_w, x_w) -
                   atan2f(ARM_L2 * sinf(j2_rad),
                          ARM_L1 + ARM_L2 * cosf(j2_rad));

    angles_out->j1 = Arm_NormalizeAngle(j1_rad * RAD_2_DEGREE);
    angles_out->j2 = Arm_NormalizeAngle(j2_rad * RAD_2_DEGREE);
    angles_out->j3 = 0.0f;

    return 1;
}

/* ===================== 关节限位检查 ===================== */

uint8_t Arm_CheckJointLimits(Arm_JointAngles_t angles)
{
    if (angles.j1 < ARM_J1_MIN_DEG || angles.j1 > ARM_J1_MAX_DEG) return 0;
    if (angles.j2 < ARM_J2_MIN_DEG || angles.j2 > ARM_J2_MAX_DEG) return 0;
    if (angles.j3 < ARM_J3_MIN_DEG || angles.j3 > ARM_J3_MAX_DEG) return 0;
    return 1;
}

/* ===================== 角度归一化 ===================== */

float Arm_NormalizeAngle(float angle_deg)
{
    float a = fmodf(angle_deg, 360.0f);
    if (a >  180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}
