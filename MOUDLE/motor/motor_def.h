/**
 * @file motor_def.h
 * @author neozng
 * @brief  电机通用的数据结构定义 - 适配STM32H7 FDCAN
 * @version 1.0
 * @date 2026-01-31
 *
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "controller.h"
#include "stdint.h"
#include "bsp_fdcan.h"

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/**
 * @brief 闭环类型,如果需要多个闭环,则使用或运算
 *        例如需要速度环和电流环: CURRENT_LOOP|SPEED_LOOP
 */
typedef enum {
    OPEN_LOOP    = 0000,
    CURRENT_LOOP = 0001,
    SPEED_LOOP   = 0010,
    ANGLE_LOOP   = 0100,
    TORQUE_LOOP  = 1000,
    // only for checking
    SPEED_AND_CURRENT_LOOP = 0011,
    ANGLE_AND_SPEED_LOOP   = 0110,
    ALL_THREE_LOOP         = 0111,
} Closeloop_Type_e;

typedef enum {
    FEEDFORWARD_NONE              = 00,
    CURRENT_FEEDFORWARD           = 01,
    SPEED_FEEDFORWARD             = 10,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

/* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/
typedef enum {
    MOTOR_FEED = 0,
    OTHER_FEED,
} Feedback_Source_e;

/* 电机正反转标志 */
typedef enum {
    MOTOR_DIRECTION_NORMAL  = 0,
    MOTOR_DIRECTION_REVERSE = 1
} Motor_Reverse_Flag_e;

/* 反馈量正反标志 */
typedef enum {
    FEEDBACK_DIRECTION_NORMAL  = 0,
    FEEDBACK_DIRECTION_REVERSE = 1
} Feedback_Reverse_Flag_e;

/* 电机启停标志 */
typedef enum {
    MOTOR_STOP    = 0,
    MOTOR_ENALBED = 1,
} Motor_Working_Type_e;

/* 电机类型 */
typedef enum {
    MOTOR_TYPE_NONE = 0,
    M2006,
    M3508,
    GM6020,
    DM4310,
    DM6006,
    DM8009,
} Motor_Type_e;

/* 电机是否使用斜坡函数 */
typedef enum {
    MOTOR_RAMP_DISABLE = 0,
    MOTOR_RAMP_ENABLE  = 1,
} Motor_Ramp_Flag_e;

/* 电机控制设置,包括闭环类型,反转标志和反馈来源 */
typedef struct
{
    Closeloop_Type_e outer_loop_type;              // 最外层的闭环,未设置时默认为最高级的闭环
    Closeloop_Type_e close_loop_type;              // 使用几个闭环(串级)
    Motor_Reverse_Flag_e motor_reverse_flag;       // 是否反转
    Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
    Feedback_Source_e angle_feedback_source;       // 角度反馈类型
    Feedback_Source_e speed_feedback_source;       // 速度反馈类型
    Feedfoward_Type_e feedforward_flag;            // 前馈标志
    Motor_Ramp_Flag_e angle_ramp_flag;             // 角度斜坡标志
    Motor_Ramp_Flag_e speed_ramp_flag;             // 速度斜坡标志
} Motor_Control_Setting_s;

/* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
typedef struct
{
    PID_Instance angle_PID;
    PID_Instance speed_PID;
    PID_Instance current_PID;

    float *other_angle_feedback_ptr; 
    float *other_speed_feedback_ptr;
    float *current_feedforward_ptr;
    float *speed_feedforward_ptr;

    float pid_ref;
} Motor_Controller_s;

/* 电机控制器初始化参数 */
typedef struct
{
    PID_Init_Config_s current_PID;
    PID_Init_Config_s speed_PID;
    PID_Init_Config_s angle_PID;

    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *current_feedforward_ptr;
    float *speed_feedforward_ptr;
} Motor_Controller_Init_s;

/* 电机初始化结构体 */
typedef struct
{
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config;
    Motor_Type_e motor_type;
    FDCAN_Init_Config_s can_init_config;
} Motor_Init_Config_s;

#endif // MOTOR_DEF_H
