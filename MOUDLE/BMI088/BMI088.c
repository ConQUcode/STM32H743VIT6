#include "BMI088.h"
#include <string.h>

#define BMI088_SPI_TIMEOUT_MS        100U

#define BMI088_ACC_CHIP_ID_REG       0x00U
#define BMI088_ACC_CHIP_ID_VALUE     0x1EU
#define BMI088_ACC_X_LSB_REG         0x12U
#define BMI088_ACC_TEMP_MSB_REG      0x22U
#define BMI088_ACC_CONF_REG          0x40U
#define BMI088_ACC_RANGE_REG         0x41U
#define BMI088_ACC_PWR_CONF_REG      0x7CU
#define BMI088_ACC_PWR_CTRL_REG      0x7DU
#define BMI088_ACC_SOFTRESET_REG     0x7EU
#define BMI088_ACC_SOFTRESET_CMD     0xB6U
#define BMI088_ACC_PWR_ACTIVE        0x00U
#define BMI088_ACC_ENABLE            0x04U
#define BMI088_ACC_CONF_DEFAULT      0xA8U
#define BMI088_ACC_RANGE_6G          0x01U

#define BMI088_GYRO_CHIP_ID_REG      0x00U
#define BMI088_GYRO_CHIP_ID_VALUE    0x0FU
#define BMI088_GYRO_X_LSB_REG        0x02U
#define BMI088_GYRO_RANGE_REG        0x0FU
#define BMI088_GYRO_BANDWIDTH_REG    0x10U
#define BMI088_GYRO_LPM1_REG         0x11U
#define BMI088_GYRO_SOFTRESET_REG    0x14U
#define BMI088_GYRO_SOFTRESET_CMD    0xB6U
#define BMI088_GYRO_NORMAL_MODE      0x00U
#define BMI088_GYRO_RANGE_2000DPS    0x00U
#define BMI088_GYRO_BW_100HZ_32HZ    0x07U

#define BMI088_READ_FLAG             0x80U
#define BMI088_WRITE_FLAG            0x7FU
#define BMI088_GRAVITY               9.80665f
#define BMI088_PI                    3.14159265358979323846f
#define BMI088_ACC_LSB_TO_MPS2       ((6.0f * BMI088_GRAVITY) / 32768.0f)
#define BMI088_GYRO_LSB_TO_RADPS     (((2000.0f * BMI088_PI) / 180.0f) / 32768.0f)

volatile BMI088_Status_t g_bmi088_status = BMI088_ERR_VERIFY;
volatile uint8_t g_bmi088_accel_id = 0U;
volatile uint8_t g_bmi088_gyro_id = 0U;
BMI088_Data_t g_bmi088_data = {0};

static void BMI088_CS_AllHigh(void)
{
    BMI088_ACC_CS_HIGH();
    BMI088_GYRO_CS_HIGH();
}

static BMI088_Status_t BMI088_SPI_TxRx(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (HAL_SPI_TransmitReceive(&hspi4, tx, rx, len, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        BMI088_CS_AllHigh();
        return BMI088_ERR_SPI;
    }

    return BMI088_OK;
}

static BMI088_Status_t BMI088_ACC_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t tx[10] = {0};
    uint8_t rx[10] = {0};
    BMI088_Status_t status;

    if ((data == NULL) || (len == 0U) || (len > 8U))
    {
        return BMI088_ERR_NULL;
    }

    tx[0] = reg | BMI088_READ_FLAG;
    BMI088_CS_AllHigh();
    BMI088_ACC_CS_LOW();
    status = BMI088_SPI_TxRx(tx, rx, (uint16_t)(len + 2U));
    BMI088_ACC_CS_HIGH();

    if (status != BMI088_OK)
    {
        return status;
    }

    memcpy(data, &rx[2], len);
    return BMI088_OK;
}

static BMI088_Status_t BMI088_ACC_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg & BMI088_WRITE_FLAG, data};
    uint8_t rx[2] = {0};
    BMI088_Status_t status;

    BMI088_CS_AllHigh();
    BMI088_ACC_CS_LOW();
    status = BMI088_SPI_TxRx(tx, rx, 2U);
    BMI088_ACC_CS_HIGH();

    return status;
}

static BMI088_Status_t BMI088_GYRO_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t tx[9] = {0};
    uint8_t rx[9] = {0};
    BMI088_Status_t status;

    if ((data == NULL) || (len == 0U) || (len > 8U))
    {
        return BMI088_ERR_NULL;
    }

    tx[0] = reg | BMI088_READ_FLAG;
    BMI088_CS_AllHigh();
    BMI088_GYRO_CS_LOW();
    status = BMI088_SPI_TxRx(tx, rx, (uint16_t)(len + 1U));
    BMI088_GYRO_CS_HIGH();

    if (status != BMI088_OK)
    {
        return status;
    }

    memcpy(data, &rx[1], len);
    return BMI088_OK;
}

static BMI088_Status_t BMI088_GYRO_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg & BMI088_WRITE_FLAG, data};
    uint8_t rx[2] = {0};
    BMI088_Status_t status;

    BMI088_CS_AllHigh();
    BMI088_GYRO_CS_LOW();
    status = BMI088_SPI_TxRx(tx, rx, 2U);
    BMI088_GYRO_CS_HIGH();

    return status;
}

static BMI088_Status_t BMI088_ACC_VerifyReg(uint8_t reg, uint8_t expected)
{
    uint8_t value = 0U;
    BMI088_Status_t status = BMI088_ACC_ReadRegs(reg, &value, 1U);

    if (status != BMI088_OK)
    {
        return status;
    }

    return (value == expected) ? BMI088_OK : BMI088_ERR_VERIFY;
}

static BMI088_Status_t BMI088_GYRO_VerifyReg(uint8_t reg, uint8_t expected)
{
    uint8_t value = 0U;
    BMI088_Status_t status = BMI088_GYRO_ReadRegs(reg, &value, 1U);

    if (status != BMI088_OK)
    {
        return status;
    }

    return (value == expected) ? BMI088_OK : BMI088_ERR_VERIFY;
}

static BMI088_Status_t BMI088_ConfigAccel(void)
{
    BMI088_Status_t status;

    status = BMI088_ACC_WriteReg(BMI088_ACC_PWR_CONF_REG, BMI088_ACC_PWR_ACTIVE);
    if (status != BMI088_OK) return status;
    HAL_Delay(5U);

    status = BMI088_ACC_WriteReg(BMI088_ACC_PWR_CTRL_REG, BMI088_ACC_ENABLE);
    if (status != BMI088_OK) return status;
    HAL_Delay(5U);

    status = BMI088_ACC_WriteReg(BMI088_ACC_CONF_REG, BMI088_ACC_CONF_DEFAULT);
    if (status != BMI088_OK) return status;
    HAL_Delay(1U);

    status = BMI088_ACC_WriteReg(BMI088_ACC_RANGE_REG, BMI088_ACC_RANGE_6G);
    if (status != BMI088_OK) return status;
    HAL_Delay(1U);

    status = BMI088_ACC_VerifyReg(BMI088_ACC_PWR_CONF_REG, BMI088_ACC_PWR_ACTIVE);
    if (status != BMI088_OK) return status;

    status = BMI088_ACC_VerifyReg(BMI088_ACC_PWR_CTRL_REG, BMI088_ACC_ENABLE);
    if (status != BMI088_OK) return status;

    status = BMI088_ACC_VerifyReg(BMI088_ACC_CONF_REG, BMI088_ACC_CONF_DEFAULT);
    if (status != BMI088_OK) return status;

    return BMI088_ACC_VerifyReg(BMI088_ACC_RANGE_REG, BMI088_ACC_RANGE_6G);
}

static BMI088_Status_t BMI088_ConfigGyro(void)
{
    BMI088_Status_t status;

    status = BMI088_GYRO_WriteReg(BMI088_GYRO_LPM1_REG, BMI088_GYRO_NORMAL_MODE);
    if (status != BMI088_OK) return status;
    HAL_Delay(30U);

    status = BMI088_GYRO_WriteReg(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000DPS);
    if (status != BMI088_OK) return status;
    HAL_Delay(1U);

    status = BMI088_GYRO_WriteReg(BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_BW_100HZ_32HZ);
    if (status != BMI088_OK) return status;
    HAL_Delay(1U);

    status = BMI088_GYRO_VerifyReg(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000DPS);
    if (status != BMI088_OK) return status;

    return BMI088_GYRO_VerifyReg(BMI088_GYRO_BANDWIDTH_REG, BMI088_GYRO_BW_100HZ_32HZ);
}

BMI088_Status_t BMI088_ReadChipIDs(uint8_t *accel_id, uint8_t *gyro_id)
{
    BMI088_Status_t status;

    if ((accel_id == NULL) || (gyro_id == NULL))
    {
        return BMI088_ERR_NULL;
    }

    status = BMI088_ACC_ReadRegs(BMI088_ACC_CHIP_ID_REG, accel_id, 1U);
    if (status != BMI088_OK)
    {
        return status;
    }

    status = BMI088_GYRO_ReadRegs(BMI088_GYRO_CHIP_ID_REG, gyro_id, 1U);
    if (status != BMI088_OK)
    {
        return status;
    }

    g_bmi088_accel_id = *accel_id;
    g_bmi088_gyro_id = *gyro_id;
    return BMI088_OK;
}

BMI088_Status_t BMI088_ReadAccelRaw(BMI088_Raw3_t *raw)
{
    uint8_t data[6] = {0};
    BMI088_Status_t status;

    if (raw == NULL)
    {
        return BMI088_ERR_NULL;
    }

    status = BMI088_ACC_ReadRegs(BMI088_ACC_X_LSB_REG, data, 6U);
    if (status != BMI088_OK)
    {
        return status;
    }

    raw->x = (int16_t)((uint16_t)data[1] << 8 | data[0]);
    raw->y = (int16_t)((uint16_t)data[3] << 8 | data[2]);
    raw->z = (int16_t)((uint16_t)data[5] << 8 | data[4]);

    return BMI088_OK;
}

BMI088_Status_t BMI088_ReadGyroRaw(BMI088_Raw3_t *raw)
{
    uint8_t data[6] = {0};
    BMI088_Status_t status;

    if (raw == NULL)
    {
        return BMI088_ERR_NULL;
    }

    status = BMI088_GYRO_ReadRegs(BMI088_GYRO_X_LSB_REG, data, 6U);
    if (status != BMI088_OK)
    {
        return status;
    }

    raw->x = (int16_t)((uint16_t)data[1] << 8 | data[0]);
    raw->y = (int16_t)((uint16_t)data[3] << 8 | data[2]);
    raw->z = (int16_t)((uint16_t)data[5] << 8 | data[4]);

    return BMI088_OK;
}

BMI088_Status_t BMI088_ReadTemperature(float *temperature_c)
{
    uint8_t data[2] = {0};
    int16_t temp_raw;
    BMI088_Status_t status;

    if (temperature_c == NULL)
    {
        return BMI088_ERR_NULL;
    }

    status = BMI088_ACC_ReadRegs(BMI088_ACC_TEMP_MSB_REG, data, 2U);
    if (status != BMI088_OK)
    {
        return status;
    }

    temp_raw = (int16_t)(((uint16_t)data[0] << 3) | ((uint16_t)data[1] >> 5));
    if (temp_raw > 1023)
    {
        temp_raw -= 2048;
    }

    *temperature_c = (float)temp_raw * 0.125f + 23.0f;
    return BMI088_OK;
}

BMI088_Status_t BMI088_ReadAccel(BMI088_Vec3f_t *accel_mps2)
{
    BMI088_Raw3_t raw;
    BMI088_Status_t status;

    if (accel_mps2 == NULL)
    {
        return BMI088_ERR_NULL;
    }

    status = BMI088_ReadAccelRaw(&raw);
    if (status != BMI088_OK)
    {
        return status;
    }

    accel_mps2->x = (float)raw.x * BMI088_ACC_LSB_TO_MPS2;
    accel_mps2->y = (float)raw.y * BMI088_ACC_LSB_TO_MPS2;
    accel_mps2->z = (float)raw.z * BMI088_ACC_LSB_TO_MPS2;

    return BMI088_OK;
}

BMI088_Status_t BMI088_ReadGyro(BMI088_Vec3f_t *gyro_rads)
{
    BMI088_Raw3_t raw;
    BMI088_Status_t status;

    if (gyro_rads == NULL)
    {
        return BMI088_ERR_NULL;
    }

    status = BMI088_ReadGyroRaw(&raw);
    if (status != BMI088_OK)
    {
        return status;
    }

    gyro_rads->x = (float)raw.x * BMI088_GYRO_LSB_TO_RADPS;
    gyro_rads->y = (float)raw.y * BMI088_GYRO_LSB_TO_RADPS;
    gyro_rads->z = (float)raw.z * BMI088_GYRO_LSB_TO_RADPS;

    return BMI088_OK;
}

BMI088_Status_t BMI088_ReadAll(BMI088_Data_t *data)
{
    BMI088_Status_t status;

    if (data == NULL)
    {
        return BMI088_ERR_NULL;
    }

    status = BMI088_ReadChipIDs(&data->accel_chip_id, &data->gyro_chip_id);
    if (status != BMI088_OK) return status;

    status = BMI088_ReadAccelRaw(&data->accel_raw);
    if (status != BMI088_OK) return status;

    status = BMI088_ReadGyroRaw(&data->gyro_raw);
    if (status != BMI088_OK) return status;

    data->accel_mps2.x = (float)data->accel_raw.x * BMI088_ACC_LSB_TO_MPS2;
    data->accel_mps2.y = (float)data->accel_raw.y * BMI088_ACC_LSB_TO_MPS2;
    data->accel_mps2.z = (float)data->accel_raw.z * BMI088_ACC_LSB_TO_MPS2;
    data->gyro_rads.x = (float)data->gyro_raw.x * BMI088_GYRO_LSB_TO_RADPS;
    data->gyro_rads.y = (float)data->gyro_raw.y * BMI088_GYRO_LSB_TO_RADPS;
    data->gyro_rads.z = (float)data->gyro_raw.z * BMI088_GYRO_LSB_TO_RADPS;

    return BMI088_ReadTemperature(&data->temperature_c);
}

BMI088_Status_t BMI088_Init(void)
{
    uint8_t accel_id = 0U;
    uint8_t gyro_id = 0U;
    BMI088_Status_t status;

    BMI088_CS_AllHigh();
    HAL_Delay(10U);

    (void)BMI088_ReadChipIDs(&accel_id, &gyro_id);
    HAL_Delay(1U);

    status = BMI088_ReadChipIDs(&accel_id, &gyro_id);
    if (status != BMI088_OK)
    {
        g_bmi088_status = status;
        return status;
    }

    if (accel_id != BMI088_ACC_CHIP_ID_VALUE)
    {
        g_bmi088_status = BMI088_ERR_ACC_ID;
        return BMI088_ERR_ACC_ID;
    }

    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        g_bmi088_status = BMI088_ERR_GYRO_ID;
        return BMI088_ERR_GYRO_ID;
    }

    status = BMI088_ACC_WriteReg(BMI088_ACC_SOFTRESET_REG, BMI088_ACC_SOFTRESET_CMD);
    if (status != BMI088_OK)
    {
        g_bmi088_status = status;
        return status;
    }
    HAL_Delay(50U);

    status = BMI088_GYRO_WriteReg(BMI088_GYRO_SOFTRESET_REG, BMI088_GYRO_SOFTRESET_CMD);
    if (status != BMI088_OK)
    {
        g_bmi088_status = status;
        return status;
    }
    HAL_Delay(50U);

    status = BMI088_ReadChipIDs(&accel_id, &gyro_id);
    if (status != BMI088_OK)
    {
        g_bmi088_status = status;
        return status;
    }

    if (accel_id != BMI088_ACC_CHIP_ID_VALUE)
    {
        g_bmi088_status = BMI088_ERR_ACC_ID;
        return BMI088_ERR_ACC_ID;
    }

    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        g_bmi088_status = BMI088_ERR_GYRO_ID;
        return BMI088_ERR_GYRO_ID;
    }

    status = BMI088_ConfigAccel();
    if (status != BMI088_OK)
    {
        g_bmi088_status = status;
        return status;
    }

    status = BMI088_ConfigGyro();
    if (status != BMI088_OK)
    {
        g_bmi088_status = status;
        return status;
    }

    status = BMI088_ReadAll(&g_bmi088_data);
    g_bmi088_status = status;

    return status;
}
