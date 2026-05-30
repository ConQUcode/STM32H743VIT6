#ifndef  HEmotor_H
#define  HEmotor_H

#include "bsp_usart.h"
#include "daemon.h"
#include "stdlib.h"
#include "string.h"
#include "stdint.h"

#define HE_MOTOR_CNT 254 // 最大支持的舵机数量
#define HE_MAX_BUFFSIZE 32

/* 指令定义 */
#define SERVO_MOVE_TIME_WRITE 1 // 位置/时间写入指令，其他指令自行添加

/* 运行状态 */
#define HE_STOP 0
#define HE_ENABLED 1

typedef struct {
    uint8_t id;      // 舵机ID (0~253, 254为广播)
    uint8_t cmd;     // 当前指令
} HEMotor_Config_s;

typedef struct {
    uint16_t position; // 目标角度 (0~1000)
    uint16_t time;     // 运行时间 (0~30000ms)
    uint8_t stop_flag; // 启停标志
} HEMotor_Ref_s;

#pragma pack(1)
typedef struct {
    USART_Instance *usart_instance;
    Daemon_Instance *daemon_instance;
    
    HEMotor_Config_s config;
    HEMotor_Ref_s ref;
    
    uint8_t send_buff[HE_MAX_BUFFSIZE];
} HEMotor_Instance;
#pragma pack()

typedef struct {
	  UART_HandleTypeDef *huart; 
    USART_Init_Config_s usart_config;
    HEMotor_Config_s motor_config;
    HEMotor_Ref_s motor_ref;
} HEMotor_Init_Config_s;

/**
 * @brief 初始化舵机实例
 */
HEMotor_Instance *HEMotorInit(HEMotor_Init_Config_s *config);

/**
 * @brief 舵机控制任务，遍历所有实例并发送指令
 */
void HEMotorControl(void);

/**
 * @brief 发送位置控制指令 (SERVO_MOVE_TIME_WRITE)
 */
void HEMotorMoveTimeWrite(HEMotor_Instance *motor, uint16_t pos, uint16_t time);


#endif