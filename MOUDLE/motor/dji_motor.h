/**
 * @file dji_motor.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   DJI电机 - 适配STM32H7 FDCAN
 * @version 1.0
 * @date 2026-01-31
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_fdcan.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define DJI_MOTOR_CNT 12 // DJI电机数量

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF   0.85f     // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f      // 必须大于0.9
#define ECD_ANGLE_COEF_DJI  0.043945f // (360/8192),将编码器值转化为角度制

/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度
    float speed_aps;          // 角速度,单位为:度/秒
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度 Celsius

    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
	
		float zero_offset; // 新增的零点偏移量
} DJI_Motor_Measure_s;

typedef struct
{
    DJI_Motor_Measure_s measure;            // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器

    FDCAN_Instance *motor_fdcan_instance; // 电机FDCAN实例
    // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;

    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志

    Daemon_Instance *daemon;
    uint32_t feed_cnt;
    float dt;
} DJIMotor_Instance;


/**
 * @brief 调用此函数注册一个DJI智能电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小DJIMotorControl()任务的运行频率.
 *
 * @attention M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return DJIMotorInstance*
 */
DJIMotor_Instance *DJIMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 修改电机启动标志
 *
 * @param motor 电机实例指针
 */
void DJIMotorEnable(DJIMotor_Instance *motor);

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 *
 */
void DJIMotorStop(DJIMotor_Instance *motor);

/**
 * @brief 切换反馈的目标来源,如将角速度和角度的来源换为IMU(小陀螺模式常用)
 *
 * @param motor 要切换反馈数据来源的电机
 * @param loop  要切换反馈数据来源的控制闭环
 * @param type  目标反馈模式
 * @param ptr   目标反馈数据指针
 */
void DJIMotorChangeFeed(DJIMotor_Instance *motor, Closeloop_Type_e loop, Feedback_Source_e type, float *ptr);

/**
 * @brief 修改电机闭环目标(外层闭环)
 *
 * @param motor  要修改的电机实例指针
 * @param outer_loop 外层闭环类型
 */
void DJIMotorOuterLoop(DJIMotor_Instance *motor, Closeloop_Type_e outer_loop);

/**
 * @brief 被application层的应用调用,给电机设定参考值.
 *        对于应用,可以将电机视为传递函数为1的设备,不需要关心底层的闭环
 *
 * @param motor 要设置的电机
 * @param ref 设定参考值
 */
void DJIMotorSetRef(DJIMotor_Instance *motor, float ref);

/**
	* @brief 重置电机里的各种回馈值
 *        
 *
 * @param motor 要设置的电机
 */
void DJIMotorReset(DJIMotor_Instance *motor);

/**
 * @brief 为所有电机实例计算三环PID,发送控制报文，
 *        该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void DJIMotorControl(void);

#endif // DJI_MOTOR_H
