#ifndef __BMI088_H
#define __BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"
#include <stdint.h>

#define BMI088_ACC_CS_LOW()    HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_RESET)
#define BMI088_ACC_CS_HIGH()   HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_SET)
#define BMI088_GYRO_CS_LOW()   HAL_GPIO_WritePin(SPI4_CS2_GPIO_Port, SPI4_CS2_Pin, GPIO_PIN_RESET)
#define BMI088_GYRO_CS_HIGH()  HAL_GPIO_WritePin(SPI4_CS2_GPIO_Port, SPI4_CS2_Pin, GPIO_PIN_SET)

typedef enum
{
    BMI088_OK = 0,
    BMI088_ERR_SPI,
    BMI088_ERR_ACC_ID,
    BMI088_ERR_GYRO_ID,
    BMI088_ERR_NULL,
    BMI088_ERR_VERIFY
} BMI088_Status_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} BMI088_Raw3_t;

typedef struct
{
    float x;
    float y;
    float z;
} BMI088_Vec3f_t;

typedef struct
{
    BMI088_Raw3_t accel_raw;
    BMI088_Raw3_t gyro_raw;
    BMI088_Vec3f_t accel_mps2;
    BMI088_Vec3f_t gyro_rads;
    float temperature_c;
    uint8_t accel_chip_id;
    uint8_t gyro_chip_id;
} BMI088_Data_t;

extern volatile BMI088_Status_t g_bmi088_status;
extern volatile uint8_t g_bmi088_accel_id;
extern volatile uint8_t g_bmi088_gyro_id;
extern BMI088_Data_t g_bmi088_data;

BMI088_Status_t BMI088_Init(void);
BMI088_Status_t BMI088_ReadChipIDs(uint8_t *accel_id, uint8_t *gyro_id);
BMI088_Status_t BMI088_ReadAccelRaw(BMI088_Raw3_t *raw);
BMI088_Status_t BMI088_ReadGyroRaw(BMI088_Raw3_t *raw);
BMI088_Status_t BMI088_ReadTemperature(float *temperature_c);
BMI088_Status_t BMI088_ReadAccel(BMI088_Vec3f_t *accel_mps2);
BMI088_Status_t BMI088_ReadGyro(BMI088_Vec3f_t *gyro_rads);
BMI088_Status_t BMI088_ReadAll(BMI088_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif
