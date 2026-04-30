/**
 * @file robot_def.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   �����˶���,���������˵ĸ��ֲ���
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

// �����˼��γߴ綨��
// ��410mm, ��320mm. ���賤Ϊǰ����(���), ��Ϊ���ҷ���(�־�)
#define CHASSIS_WHEEL_BASE      0.410f      // ���(ǰ���־���, X�᷽��)
#define CHASSIS_WHEEL_TRACK     0.320f      // �־�(�����־���, Y�᷽��)
#define CHASSIS_HALF_BASE       (CHASSIS_WHEEL_BASE / 2.0f)
#define CHASSIS_HALF_TRACK      (CHASSIS_WHEEL_TRACK / 2.0f)

// �����ζ��ָ���������ĵ��������� (��λ: ����ף�����730/816/816����ó�)
// 3���֣����� (��ǰ)
#define W3_X  (0.4865f)
#define W3_Y  (0.0f)   

// 1���֣��ױ��󶥵� (���) -> ����1Ϊ��2Ϊ��
#define W1_X  (-0.2433f)
#define W1_Y  (-0.3650f)

// 2���֣��ױ��Ҷ��� (�Һ�)
#define W2_X  (-0.2433f)
#define W2_Y  (0.3650f)

// ����ٶ�ת����غ궨�� 
#define MOTOR_REDUCTION_RATIO    19.2032f     // ������ٱ� 1:19.2302
#define WHEEL_RADIUS_M           0.081f     // ���Ӱ뾶(��) - ����ʵ�����ӳߴ����

// ���ٶ�ת��Ϊ3508����ٶȻ�������ת��ϵ��
// ת����ʽ: ���RPM = ���ٶ�(m/s) * ת��ϵ��
// �Ƶ�����:
// 1. ����ת��(rpm) = ���ٶ�(m/s) * 60 / (2 * �� * ���Ӱ뾶)
// 2. ���ת��(rpm) = ����ת��(rpm) * ���ٱ�
// 3. ת��ϵ�� = 60 * ���ٱ� / (2 * �� * ���Ӱ뾶)
#define LINEAR_VELOCITY_TO_MOTOR_RPM    (60.0f * MOTOR_REDUCTION_RATIO / (2.0f * 3.14159f * WHEEL_RADIUS_M)) // Լ���� 8732.24

// ����ת�����ӵ���ٶ�ת��Ϊ���ٶ� (�����ٶȷ�����ʾ��)
#define MOTOR_SPEED_TO_LINEAR_VEL(motor_speed)   ((motor_speed) / LINEAR_VELOCITY_TO_MOTOR_RPM)

// DJ6020������ٶ�ת����غ궨��
#define GM6020_REDUCTION_RATIO    1.0f        // GM6020ͨ��Ϊֱ�������ٱ�1:1
#define GM6020_ECD_RANGE          8192.0f     // ��������Χ 0-8191

// ���ٶ�(rad/s)ת��ΪGM6020���RPM
// ת����ʽ: ���RPM = ���ٶ�(rad/s) * 60 / (2 * ��) * ���ٱ�
#define ANGULAR_VELOCITY_TO_GM6020_RPM    (60.0f * GM6020_REDUCTION_RATIO / (2.0f * 3.14159f))

// ����ת������GM6020���RPMת��Ϊ���ٶ�(rad/s)
#define GM6020_RPM_TO_ANGULAR_VEL(rpm)    ((rpm) / ANGULAR_VELOCITY_TO_GM6020_RPM)


#define STEERING_CHASSIS_ALIGN_ECD_LF   5800// ���� A 4������ֵ�����л�е�Ķ���Ҫ�޸�7848-
#define STEERING_CHASSIS_ALIGN_ECD_LB   5610 // ���� B 2������ֵ�����л�е�Ķ���Ҫ�޸�3562+
#define STEERING_CHASSIS_ALIGN_ECD_RF   1734// ���� C 1������ֵ�����л�е�Ķ���Ҫ�޸�7878+
#define STEERING_CHASSIS_ALIGN_ECD_RB   4389 // ���� D 3������ֵ�����л�е�Ķ���Ҫ�޸�6437-

#define STEERING_CHASSIS_ALIGN_ANGLE_LF STEERING_CHASSIS_ALIGN_ECD_LF / 8192.f * 360.f // ���� A ����Ƕ�
#define STEERING_CHASSIS_ALIGN_ANGLE_LB STEERING_CHASSIS_ALIGN_ECD_LB / 8192.f * 360.f // ���� B ����Ƕ�
#define STEERING_CHASSIS_ALIGN_ANGLE_RF STEERING_CHASSIS_ALIGN_ECD_RF / 8192.f * 360.f // ���� C ����Ƕ�
#define STEERING_CHASSIS_ALIGN_ANGLE_RB STEERING_CHASSIS_ALIGN_ECD_RB / 8192.f * 360.f // ���� D ����Ƕ�

// ================== �����ֶ������ (������ʵ�ʲ���ֵ) ==================
// ���裺1��Ϊ���2��Ϊ�Һ�3��Ϊǰ����
#define STEERING_CHASSIS_ALIGN_ECD_1   6900 // ������1���ֱ�����ֵ700
#define STEERING_CHASSIS_ALIGN_ECD_2   0 // ������2���ֱ�����ֵ8011
#define STEERING_CHASSIS_ALIGN_ECD_3   2270 // ������4���ֱ�����ֵ1240

#define STEERING_CHASSIS_ALIGN_ANGLE_1 STEERING_CHASSIS_ALIGN_ECD_1 / 8192.f * 360.f
#define STEERING_CHASSIS_ALIGN_ANGLE_2 STEERING_CHASSIS_ALIGN_ECD_2 / 8192.f * 360.f
#define STEERING_CHASSIS_ALIGN_ANGLE_3 STEERING_CHASSIS_ALIGN_ECD_3 / 8192.f * 360.f

#pragma pack(1) // ѹ���ṹ��,ȡ���ֽڶ���,��������ݶ����ܱ�����



#pragma pack() // ȡ��ѹ��
#endif

