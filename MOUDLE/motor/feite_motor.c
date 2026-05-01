#include "feite_motor.h"
#include <stdlib.h>
#include <string.h>

/* 默认只使用一条 USART1 飞特总线，多舵机实例共享该总线状态和收发缓冲。 */
static FeiteMotor_Bus_s default_bus;
static uint8_t default_bus_initialized = 0U;
static FeiteMotor_Instance *feite_motor_instances[FEITE_MOTOR_CNT] = {NULL};
static uint8_t feite_motor_idx = 0U;

static void FeiteSetError(FeiteMotor_Bus_s *bus, FeiteMotor_Error_e error)
{
    if (bus != NULL) {
        bus->last_error = error;
    }
}

/* 按总线配置的字节序，把主机端 16 位数据拆成舵机协议的低/高字节。 */
static void FeiteHostToServoU16(FeiteMotor_Bus_s *bus, uint8_t *data_l, uint8_t *data_h, uint16_t data)
{
    if (bus != NULL && bus->endian == FEITE_ENDIAN_BIG) {
        *data_l = (uint8_t)(data >> 8);
        *data_h = (uint8_t)(data & 0xFFU);
    } else {
        *data_h = (uint8_t)(data >> 8);
        *data_l = (uint8_t)(data & 0xFFU);
    }
}

/* 将舵机回传的两个字节还原成主机端 16 位数据，和发送端字节序保持对称。 */
static uint16_t FeiteServoToHostU16(FeiteMotor_Bus_s *bus, uint8_t data_l, uint8_t data_h)
{
    if (bus != NULL && bus->endian == FEITE_ENDIAN_BIG) {
        return (uint16_t)(((uint16_t)data_l << 8) | data_h);
    }

    return (uint16_t)(((uint16_t)data_h << 8) | data_l);
}

/* 飞特/SCS 协议校验：从 ID 到最后一个参数求和后按位取反。 */
static uint8_t FeiteCalcChecksum(const uint8_t *packet)
{
    uint8_t len;
    uint16_t sum = 0U;
    uint8_t i;

    len = packet[3];
    for (i = 2U; i < (uint8_t)(len + 3U); ++i) {
        sum += packet[i];
    }

    return (uint8_t)(~sum);
}

/* 构造标准指令帧：0xFF 0xFF ID LEN INST PARAM... CHECKSUM。 */
static uint16_t FeiteBuildPacket(uint8_t id,
                                 uint8_t instruction,
                                 const uint8_t *params,
                                 uint8_t param_len,
                                 uint8_t *out,
                                 uint16_t out_len)
{
    uint8_t i;
    uint8_t len;

    if (out == NULL || out_len < (uint16_t)(param_len + 6U) || param_len > FEITE_PACKET_MAX_PARAM_LEN) {
        return 0U;
    }

    len = (uint8_t)(param_len + 2U);
    out[0] = 0xFFU;
    out[1] = 0xFFU;
    out[2] = id;
    out[3] = len;
    out[4] = instruction;

    for (i = 0U; i < param_len; ++i) {
        out[5U + i] = params[i];
    }

    out[5U + param_len] = FeiteCalcChecksum(out);
    return (uint16_t)(param_len + 6U);
}

/* 统一发送一帧指令，所有上层写寄存器/读寄存器最终都会走这里。 */
static HAL_StatusTypeDef FeiteWritePacket(FeiteMotor_Bus_s *bus,
                                           uint8_t id,
                                           uint8_t instruction,
                                           const uint8_t *params,
                                           uint8_t param_len)
{
    uint16_t packet_len;
    HAL_StatusTypeDef status;

    if (bus == NULL || bus->huart == NULL) {
        FeiteSetError(bus, FEITE_ERROR_PARAM);
        return HAL_ERROR;
    }

    packet_len = FeiteBuildPacket(id, instruction, params, param_len, bus->tx_buf, sizeof(bus->tx_buf));
    if (packet_len == 0U) {
        FeiteSetError(bus, FEITE_ERROR_LENGTH);
        return HAL_ERROR;
    }

    status = HAL_UART_Transmit(bus->huart, bus->tx_buf, packet_len, bus->tx_timeout_ms);
    if (status != HAL_OK) {
        FeiteSetError(bus, FEITE_ERROR_HAL);
        return status;
    }

    FeiteSetError(bus, FEITE_ERROR_NONE);
    return HAL_OK;
}

static HAL_StatusTypeDef FeiteReadByte(FeiteMotor_Bus_s *bus, uint8_t *data)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Receive(bus->huart, data, 1U, bus->rx_timeout_ms);
    if (status != HAL_OK) {
        FeiteSetError(bus, FEITE_ERROR_NO_REPLY);
    }

    return status;
}

/* 从串口流中寻找连续两个 0xFF，避免前面残留杂字节导致应答错位。 */
static HAL_StatusTypeDef FeiteCheckHead(FeiteMotor_Bus_s *bus)
{
    uint8_t data;
    uint8_t last = 0U;
    uint8_t count;

    if (bus == NULL || bus->huart == NULL) {
        FeiteSetError(bus, FEITE_ERROR_PARAM);
        return HAL_ERROR;
    }

    for (count = 0U; count < 16U; ++count) {
        if (FeiteReadByte(bus, &data) != HAL_OK) {
            return HAL_TIMEOUT;
        }

        if (last == 0xFFU && data == 0xFFU) {
            return HAL_OK;
        }

        last = data;
    }

    FeiteSetError(bus, FEITE_ERROR_NO_REPLY);
    return HAL_TIMEOUT;
}

/* 读取状态应答帧：0xFF 0xFF ID LEN ERROR PARAM... CHECKSUM。 */
static HAL_StatusTypeDef FeiteReadStatus(FeiteMotor_Bus_s *bus,
                                          uint8_t expected_id,
                                          uint8_t *params,
                                          uint8_t *param_len)
{
    uint8_t header[3];
    uint8_t packet_len;
    uint8_t param_count;
    uint8_t checksum;
    uint8_t checksum_calc;
    uint8_t i;

    if (bus == NULL || params == NULL || param_len == NULL) {
        FeiteSetError(bus, FEITE_ERROR_PARAM);
        return HAL_ERROR;
    }

    if (FeiteCheckHead(bus) != HAL_OK) {
        return HAL_TIMEOUT;
    }

    if (HAL_UART_Receive(bus->huart, header, sizeof(header), bus->rx_timeout_ms) != HAL_OK) {
        FeiteSetError(bus, FEITE_ERROR_NO_REPLY);
        return HAL_TIMEOUT;
    }

    if (expected_id != FEITE_BROADCAST_ID && header[0] != expected_id) {
        FeiteSetError(bus, FEITE_ERROR_ID);
        return HAL_ERROR;
    }

    packet_len = header[1];
    if (packet_len < 2U || packet_len > (FEITE_PACKET_MAX_LEN - 4U)) {
        FeiteSetError(bus, FEITE_ERROR_LENGTH);
        return HAL_ERROR;
    }

    param_count = (uint8_t)(packet_len - 2U);
    if (param_count > FEITE_PACKET_MAX_PARAM_LEN) {
        FeiteSetError(bus, FEITE_ERROR_LENGTH);
        return HAL_ERROR;
    }

    if (param_count > 0U && HAL_UART_Receive(bus->huart, params, param_count, bus->rx_timeout_ms) != HAL_OK) {
        FeiteSetError(bus, FEITE_ERROR_NO_REPLY);
        return HAL_TIMEOUT;
    }

    if (HAL_UART_Receive(bus->huart, &checksum, 1U, bus->rx_timeout_ms) != HAL_OK) {
        FeiteSetError(bus, FEITE_ERROR_NO_REPLY);
        return HAL_TIMEOUT;
    }

    checksum_calc = (uint8_t)(header[0] + header[1] + header[2]);
    for (i = 0U; i < param_count; ++i) {
        checksum_calc += params[i];
    }
    checksum_calc = (uint8_t)(~checksum_calc);

    if (checksum != checksum_calc) {
        FeiteSetError(bus, FEITE_ERROR_CHECKSUM);
        return HAL_ERROR;
    }

    bus->last_status = header[2];
    *param_len = param_count;

    FeiteSetError(bus, FEITE_ERROR_NONE);
    return HAL_OK;
}

static HAL_StatusTypeDef FeiteAck(FeiteMotor_Bus_s *bus, uint8_t id)
{
    uint8_t params[FEITE_PACKET_MAX_PARAM_LEN];
    uint8_t param_len = 0U;

    if (id == FEITE_BROADCAST_ID || bus == NULL || bus->reply_level == 0U) {
        return HAL_OK;
    }

    return FeiteReadStatus(bus, id, params, &param_len);
}

/* 写寄存器基础接口，data 会被拼在寄存器地址后作为 WRITE 指令参数。 */
static HAL_StatusTypeDef FeiteWriteReg(FeiteMotor_Instance *motor,
                                        uint8_t reg,
                                        const uint8_t *data,
                                        uint8_t len)
{
    uint8_t params[FEITE_PACKET_MAX_PARAM_LEN];

    if (motor == NULL || motor->bus == NULL || data == NULL || len > (FEITE_PACKET_MAX_PARAM_LEN - 1U)) {
        if (motor != NULL) {
            FeiteSetError(motor->bus, FEITE_ERROR_PARAM);
        }
        return HAL_ERROR;
    }

    params[0] = reg;
    memcpy(&params[1], data, len);

    if (FeiteWritePacket(motor->bus, motor->id, FEITE_INST_WRITE, params, (uint8_t)(len + 1U)) != HAL_OK) {
        return HAL_ERROR;
    }

    return FeiteAck(motor->bus, motor->id);
}

/* 读寄存器基础接口，成功时 out 中只保留参数数据，不包含状态字节和校验。 */
static HAL_StatusTypeDef FeiteReadReg(FeiteMotor_Instance *motor,
                                       uint8_t reg,
                                       uint8_t len,
                                       uint8_t *out)
{
    uint8_t params[2];
    uint8_t rx_len = 0U;

    if (motor == NULL || motor->bus == NULL || out == NULL || len == 0U || len > FEITE_PACKET_MAX_PARAM_LEN) {
        if (motor != NULL) {
            FeiteSetError(motor->bus, FEITE_ERROR_PARAM);
        }
        return HAL_ERROR;
    }

    params[0] = reg;
    params[1] = len;

    if (FeiteWritePacket(motor->bus, motor->id, FEITE_INST_READ, params, sizeof(params)) != HAL_OK) {
        return HAL_ERROR;
    }

    if (FeiteReadStatus(motor->bus, motor->id, out, &rx_len) != HAL_OK) {
        return HAL_ERROR;
    }

    if (rx_len != len) {
        FeiteSetError(motor->bus, FEITE_ERROR_LENGTH);
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* 初始化 USART 总线；config 为空时默认绑定 CubeMX 已初始化的 huart1。 */
FeiteMotor_Bus_s *FeiteMotorBusInit(FeiteMotor_Bus_Init_Config_s *config)
{
    FeiteMotor_Bus_s *bus = &default_bus;

    if (config == NULL && default_bus_initialized) {
        return bus;
    }

    memset(bus, 0, sizeof(FeiteMotor_Bus_s));

    bus->huart = (config != NULL && config->huart != NULL) ? config->huart : &huart1;
    bus->tx_timeout_ms = (config != NULL && config->tx_timeout_ms != 0U) ? config->tx_timeout_ms : FEITE_DEFAULT_TIMEOUT_MS;
    bus->rx_timeout_ms = (config != NULL && config->rx_timeout_ms != 0U) ? config->rx_timeout_ms : FEITE_DEFAULT_TIMEOUT_MS;
    bus->endian = (config != NULL) ? config->endian : FEITE_ENDIAN_LITTLE;
    bus->reply_level = (config != NULL) ? config->reply_level : 1U;
    bus->last_error = FEITE_ERROR_NONE;
    default_bus_initialized = 1U;

    return bus;
}

/* 注册一个飞特舵机实例，上层之后通过返回的句柄设置目标和读取反馈。 */
FeiteMotor_Instance *FeiteMotorInit(FeiteMotor_Init_Config_s *config)
{
    FeiteMotor_Instance *motor;

    if (config == NULL || config->id == FEITE_BROADCAST_ID || feite_motor_idx >= FEITE_MOTOR_CNT) {
        return NULL;
    }

    motor = (FeiteMotor_Instance *)malloc(sizeof(FeiteMotor_Instance));
    if (motor == NULL) {
        return NULL;
    }

    memset(motor, 0, sizeof(FeiteMotor_Instance));

    motor->bus = config->bus;
    if (motor->bus == NULL) {
        motor->bus = FeiteMotorBusInit(NULL);
    }

    motor->id = config->id;
    motor->model = config->model;
    if (motor->model == FEITE_MODEL_UNKNOWN) {
        motor->model = FEITE_MODEL_HLS_SCS;
    }
    motor->ref_position = config->init_position;
    motor->ref_speed = config->init_speed;
    motor->ref_acc = config->init_acc;
    motor->ref_torque = config->init_torque;
    motor->raw_to_deg = (config->raw_to_deg != 0.0f) ? config->raw_to_deg : FEITE_DEFAULT_RAW_TO_DEG;
    motor->motor_reverse_flag = config->motor_reverse_flag;
    motor->stop_flag = MOTOR_ENALBED;

    feite_motor_instances[feite_motor_idx++] = motor;
    return motor;
}

void FeiteMotorEnable(FeiteMotor_Instance *motor)
{
    if (motor != NULL) {
        motor->stop_flag = MOTOR_ENALBED;
    }
}

void FeiteMotorStop(FeiteMotor_Instance *motor)
{
    if (motor != NULL) {
        motor->stop_flag = MOTOR_STOP;
        (void)FeiteMotorMoveTo(motor, motor->ref_position, 0U, motor->ref_acc, 0U);
    }
}

void FeiteMotorSetRef(FeiteMotor_Instance *motor, int16_t position)
{
    if (motor != NULL) {
        motor->ref_position = position;
    }
}

void FeiteMotorSetSpeed(FeiteMotor_Instance *motor, uint16_t speed)
{
    if (motor != NULL) {
        motor->ref_speed = speed;
    }
}

void FeiteMotorSetAcc(FeiteMotor_Instance *motor, uint8_t acc)
{
    if (motor != NULL) {
        motor->ref_acc = acc;
    }
}

/* HLS/SCS 位置控制：从 ACC 寄存器开始连续写 ACC、Position、Torque、Speed。 */
HAL_StatusTypeDef FeiteMotorMoveTo(FeiteMotor_Instance *motor,
                                   int16_t position,
                                   uint16_t speed,
                                   uint8_t acc,
                                   uint16_t torque)
{
    uint8_t data[7];
    uint16_t position_data;

    if (motor == NULL || motor->bus == NULL) {
        return HAL_ERROR;
    }

    if (motor->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
        position = (int16_t)(-position);
    }

    if (position < 0) {
        position_data = (uint16_t)(-position);
        position_data |= 0x8000U;
    } else {
        position_data = (uint16_t)position;
    }

    data[0] = acc;
    FeiteHostToServoU16(motor->bus, &data[1], &data[2], position_data);
    FeiteHostToServoU16(motor->bus, &data[3], &data[4], torque);
    FeiteHostToServoU16(motor->bus, &data[5], &data[6], speed);

    return FeiteWriteReg(motor, FEITE_REG_ACC, data, sizeof(data));
}

/* 读取当前位置和当前速度，并换算出带符号位置和角度值。 */
HAL_StatusTypeDef FeiteMotorReadFeedback(FeiteMotor_Instance *motor)
{
    uint8_t data[4];
    uint16_t position_raw;
    uint16_t speed_raw;

    if (motor == NULL || motor->bus == NULL) {
        return HAL_ERROR;
    }

    if (FeiteReadReg(motor, FEITE_REG_PRESENT_POSITION_L, sizeof(data), data) != HAL_OK) {
        motor->measure.online = 0U;
        return HAL_ERROR;
    }

    position_raw = FeiteServoToHostU16(motor->bus, data[0], data[1]);
    speed_raw = FeiteServoToHostU16(motor->bus, data[2], data[3]);

    motor->measure.position_raw = position_raw;
    motor->measure.position_signed = (position_raw & 0x8000U) ? -(int16_t)(position_raw & 0x7FFFU) : (int16_t)position_raw;
    motor->measure.speed_raw = (speed_raw & 0x8000U) ? -(int16_t)(speed_raw & 0x7FFFU) : (int16_t)speed_raw;
    motor->measure.angle_deg = (float)motor->measure.position_signed * motor->raw_to_deg;
    motor->measure.error = motor->bus->last_status;
    motor->measure.online = 1U;
    motor->measure.last_update_tick = HAL_GetTick();

    return HAL_OK;
}

/* 低风险在线检测入口，建议实机调试时先 Ping 再发位置控制。 */
HAL_StatusTypeDef FeiteMotorPing(FeiteMotor_Instance *motor)
{
    if (motor == NULL || motor->bus == NULL) {
        return HAL_ERROR;
    }

    if (FeiteWritePacket(motor->bus, motor->id, FEITE_INST_PING, NULL, 0U) != HAL_OK) {
        motor->measure.online = 0U;
        return HAL_ERROR;
    }

    if (FeiteAck(motor->bus, motor->id) != HAL_OK) {
        motor->measure.online = 0U;
        return HAL_ERROR;
    }

    motor->measure.online = 1U;
    motor->measure.last_update_tick = HAL_GetTick();
    return HAL_OK;
}

/* 周期控制入口：遍历已注册实例，把每个舵机的当前 ref 写到总线上。 */
void FeiteMotorControl(void)
{
    uint8_t i;
    FeiteMotor_Instance *motor;

    for (i = 0U; i < feite_motor_idx; ++i) {
        motor = feite_motor_instances[i];
        if (motor == NULL) {
            continue;
        }

        if (motor->stop_flag == MOTOR_STOP) {
            (void)FeiteMotorMoveTo(motor, motor->ref_position, 0U, motor->ref_acc, 0U);
        } else {
            (void)FeiteMotorMoveTo(motor, motor->ref_position, motor->ref_speed, motor->ref_acc, motor->ref_torque);
        }
    }
}
