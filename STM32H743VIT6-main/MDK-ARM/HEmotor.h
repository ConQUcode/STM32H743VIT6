#ifndef  HEmotor_H
#define  HEmotor_H

#include "bsp_usart.h"
#include "daemon.h"
#include "stdlib.h"
#include "string.h"
#include "stdint.h"

#define HE_MOTOR_CNT 254 // ���֧�ֵĶ������
#define HE_MAX_BUFFSIZE 32

/* ָ��� */
#define SERVO_MOVE_TIME_WRITE 1 // λ��/ʱ��д��ָ�����ָ����������

/* ����״̬ */
#define HE_STOP 0
#define HE_ENABLED 1

typedef struct {
    uint8_t id;      // ���ID (0~253, 254Ϊ�㲥)
    uint8_t cmd;     // ��ǰָ��
} HEMotor_Config_s;

typedef struct {
    uint16_t position; // Ŀ��Ƕ� (0~1000)
    uint16_t time;     // ����ʱ�� (0~30000ms)
    uint8_t stop_flag; // ��ͣ��־
} HEMotor_Ref_s;


typedef struct {
    USART_Instance *usart_instance;
    Daemon_Instance *daemon_instance;
    
    HEMotor_Config_s config;
    HEMotor_Ref_s ref;
    
    uint16_t last_position; // 上一次发送的位置
    uint16_t last_time;     // 上一次发送的时间
    
    uint8_t send_buff[HE_MAX_BUFFSIZE];
} HEMotor_Instance;


typedef struct {
	  UART_HandleTypeDef *huart; 
    USART_Init_Config_s usart_config;
    HEMotor_Config_s motor_config;
    HEMotor_Ref_s motor_ref;
} HEMotor_Init_Config_s;

/**
 * @brief ��ʼ�����ʵ��
 */
HEMotor_Instance *HEMotorInit(HEMotor_Init_Config_s *config);

/**
 * @brief ����������񣬱�������ʵ��������ָ��
 */
void HEMotorControl(void);

/**
 * @brief ����λ�ÿ���ָ�� (SERVO_MOVE_TIME_WRITE)
 */
void HEMotorMoveTimeWrite(HEMotor_Instance *motor, uint16_t pos, uint16_t time);


#endif