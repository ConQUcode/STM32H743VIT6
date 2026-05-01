#ifndef FEITE_MOTOR_H
#define FEITE_MOTOR_H

#include "motor_def.h"
#include "usart.h"
#include <stdint.h>

#define FEITE_MOTOR_CNT             16U
#define FEITE_PACKET_MAX_LEN        64U
#define FEITE_PACKET_MAX_PARAM_LEN  (FEITE_PACKET_MAX_LEN - 6U)
#define FEITE_BROADCAST_ID          0xFEU

#define FEITE_INST_PING             0x01U
#define FEITE_INST_READ             0x02U
#define FEITE_INST_WRITE            0x03U
#define FEITE_INST_REG_WRITE        0x04U
#define FEITE_INST_ACTION           0x05U
#define FEITE_INST_RESET            0x06U
#define FEITE_INST_SYNC_WRITE       0x83U

#define FEITE_REG_ID                5U
#define FEITE_REG_ACC               41U
#define FEITE_REG_GOAL_POSITION_L   42U
#define FEITE_REG_PRESENT_POSITION_L 56U
#define FEITE_REG_PRESENT_SPEED_L   58U

#define FEITE_DEFAULT_TIMEOUT_MS    20U
#define FEITE_DEFAULT_RAW_TO_DEG    (360.0f / 4096.0f)

typedef enum {
    FEITE_MODEL_UNKNOWN = 0,
    FEITE_MODEL_HLS_SCS,
} FeiteMotor_Model_e;

typedef enum {
    FEITE_ENDIAN_LITTLE = 0,
    FEITE_ENDIAN_BIG,
} FeiteMotor_Endian_e;

typedef enum {
    FEITE_ERROR_NONE = 0,
    FEITE_ERROR_NO_REPLY,
    FEITE_ERROR_CHECKSUM,
    FEITE_ERROR_ID,
    FEITE_ERROR_LENGTH,
    FEITE_ERROR_HAL,
    FEITE_ERROR_PARAM,
} FeiteMotor_Error_e;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t tx_timeout_ms;
    uint32_t rx_timeout_ms;
    FeiteMotor_Endian_e endian;
    uint8_t reply_level;
    uint8_t last_status;
    FeiteMotor_Error_e last_error;
    uint8_t tx_buf[FEITE_PACKET_MAX_LEN];
    uint8_t rx_buf[FEITE_PACKET_MAX_LEN];
} FeiteMotor_Bus_s;

typedef struct {
    uint16_t position_raw;
    int16_t position_signed;
    float angle_deg;
    int16_t speed_raw;
    uint8_t temperature;
    uint8_t error;
    uint8_t online;
    uint32_t last_update_tick;
} FeiteMotor_Measure_s;

typedef struct {
    uint8_t id;
    FeiteMotor_Model_e model;
    FeiteMotor_Bus_s *bus;
    FeiteMotor_Measure_s measure;
    int16_t ref_position;
    uint16_t ref_speed;
    uint8_t ref_acc;
    uint16_t ref_torque;
    float raw_to_deg;
    Motor_Working_Type_e stop_flag;
    Motor_Reverse_Flag_e motor_reverse_flag;
} FeiteMotor_Instance;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t tx_timeout_ms;
    uint32_t rx_timeout_ms;
    FeiteMotor_Endian_e endian;
    uint8_t reply_level;
} FeiteMotor_Bus_Init_Config_s;

typedef struct {
    FeiteMotor_Bus_s *bus;
    uint8_t id;
    FeiteMotor_Model_e model;
    int16_t init_position;
    uint16_t init_speed;
    uint8_t init_acc;
    uint16_t init_torque;
    float raw_to_deg;
    Motor_Reverse_Flag_e motor_reverse_flag;
} FeiteMotor_Init_Config_s;

FeiteMotor_Bus_s *FeiteMotorBusInit(FeiteMotor_Bus_Init_Config_s *config);
FeiteMotor_Instance *FeiteMotorInit(FeiteMotor_Init_Config_s *config);

void FeiteMotorEnable(FeiteMotor_Instance *motor);
void FeiteMotorStop(FeiteMotor_Instance *motor);
void FeiteMotorSetRef(FeiteMotor_Instance *motor, int16_t position);
void FeiteMotorSetSpeed(FeiteMotor_Instance *motor, uint16_t speed);
void FeiteMotorSetAcc(FeiteMotor_Instance *motor, uint8_t acc);

HAL_StatusTypeDef FeiteMotorMoveTo(FeiteMotor_Instance *motor,
                                   int16_t position,
                                   uint16_t speed,
                                   uint8_t acc,
                                   uint16_t torque);
HAL_StatusTypeDef FeiteMotorReadFeedback(FeiteMotor_Instance *motor);
HAL_StatusTypeDef FeiteMotorPing(FeiteMotor_Instance *motor);
void FeiteMotorControl(void);

#endif
