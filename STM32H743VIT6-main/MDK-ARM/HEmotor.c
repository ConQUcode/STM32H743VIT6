#include "HEmotor.h"

static uint8_t he_idx = 0;
static HEMotor_Instance *he_motor_instances[HE_MOTOR_CNT] = {NULL};

/**
 * @brief 守护进程回调
 */
static void HEMotorLostCallback(void *motor_ptr)
{
    HEMotor_Instance *motor = (HEMotor_Instance *)motor_ptr;
    motor->ref.stop_flag = HE_STOP;
}

HEMotor_Instance *HEMotorInit(HEMotor_Init_Config_s *config)
{
if (config == NULL || he_idx >= HE_MOTOR_CNT) return NULL;

    HEMotor_Instance *motor = (HEMotor_Instance *)malloc(sizeof(HEMotor_Instance));
    memset(motor, 0, sizeof(HEMotor_Instance));

    // 显式构造 BSP 串口初始化结构体
    USART_Init_Config_s bsp_usart_config = {
        .usart_handle = config->huart,       // 确保这里拿到了 &huart2
        .recv_buff_size = HE_MAX_BUFFSIZE,
        .module_callback = NULL
    };
    
    // 调用 BSP 注册函数
    motor->usart_instance = USARTRegister(&bsp_usart_config);

    motor->config = config->motor_config;
    motor->ref = config->motor_ref;

    // 注册守护进程
    Daemon_Init_Config_s daemon_config = {
        .callback = HEMotorLostCallback,
        .owner_id = motor,
        .reload_count = 5, // 50ms超时
    };
    motor->daemon_instance = DaemonRegister(&daemon_config);

    he_motor_instances[he_idx++] = motor;
    return motor;
}

/**
 * @brief 发送指令封装
 */
static void HEMotorSendPacket(HEMotor_Instance *motor, uint8_t cmd, uint8_t *params, uint8_t param_len)
{
    // Length = ID(1) + Length字段本身(1) + Cmd(1) + 参数(N)
    uint8_t length = param_len + 3; 
    // Total = 帧头(2) + ID(1) + Length(1) + Cmd(1) + 参数(N) + 校验和(1)
    // 也就是 Total = 2(头) + length + 1(校验和) = length + 3
    uint8_t total_len = length + 3;  
    
    uint8_t *buf = motor->send_buff;
    uint32_t checksum_total = 0;

    buf[0] = 0x55;
    buf[1] = 0x55;
    buf[2] = motor->config.id;
    buf[3] = length;
    buf[4] = cmd;

    // 校验和累加从 ID 开始：buf[2], buf[3], buf[4] ...
    checksum_total = buf[2] + buf[3] + buf[4];

    for (uint8_t i = 0; i < param_len; i++) {
        buf[5 + i] = params[i];
        checksum_total += params[i];
    }

    // Checksum = ~(累加和的低8位)
    buf[5 + param_len] = (uint8_t)(~(checksum_total & 0xFF));

    USARTSend(motor->usart_instance, buf, total_len, USART_TRANSFER_BLOCKING);
}

void HEMotorMoveTimeWrite(HEMotor_Instance *motor, uint16_t pos, uint16_t time)
{
    if (motor == NULL || motor->ref.stop_flag == HE_STOP) return;

   
    uint8_t params[4];
    params[0] = pos & 0xFF;         // 角度低位
    params[1] = (pos >> 8) & 0xFF;  // 角度高位
    params[2] = time & 0xFF;        // 时间低位
    params[3] = (time >> 8) & 0xFF; // 时间高位

    HEMotorSendPacket(motor, SERVO_MOVE_TIME_WRITE, params, 4);
}

void HEMotorControl(void)
{
    for (int i = 0; i < he_idx; i++) {
        if (he_motor_instances[i] != NULL && he_motor_instances[i]->ref.stop_flag == HE_ENABLED) {
            HEMotorMoveTimeWrite(he_motor_instances[i], 
                                 he_motor_instances[i]->ref.position, 
                                 he_motor_instances[i]->ref.time);
        }
    }
}