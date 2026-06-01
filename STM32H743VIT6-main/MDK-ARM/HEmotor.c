#include "HEmotor.h"

static uint8_t he_idx = 0;
static HEMotor_Instance *he_motor_instances[HE_MOTOR_CNT] = {NULL};

HEMotor_Instance *HEMotorInit(HEMotor_Init_Config_s *config)
{
if (config == NULL || he_idx >= HE_MOTOR_CNT) return NULL;

    HEMotor_Instance *motor = (HEMotor_Instance *)malloc(sizeof(HEMotor_Instance));
    memset(motor, 0, sizeof(HEMotor_Instance));

    // ��ʽ���� BSP ���ڳ�ʼ���ṹ��
    USART_Init_Config_s bsp_usart_config = {
        .usart_handle = config->huart,       // ȷ�������õ��� &huart2
        .recv_buff_size = HE_MAX_BUFFSIZE,
        .module_callback = NULL
    };
    
    // ���� BSP ע�ắ��
    motor->usart_instance = USARTRegister(&bsp_usart_config);

    motor->config = config->motor_config;
    motor->ref = config->motor_ref;

    he_motor_instances[he_idx++] = motor;
    return motor;
}

/**
 * @brief ����ָ���װ
 */
static void HEMotorSendPacket(HEMotor_Instance *motor, uint8_t cmd, uint8_t *params, uint8_t param_len)
{
    // Length = ID(1) + Length�ֶα���(1) + Cmd(1) + ����(N)
    uint8_t length = param_len + 3; 
    // Total = ֡ͷ(2) + ID(1) + Length(1) + Cmd(1) + ����(N) + У���(1)
    // Ҳ���� Total = 2(ͷ) + length + 1(У���) = length + 3
    uint8_t total_len = length + 3;  
    
    uint8_t *buf = motor->send_buff;
    uint32_t checksum_total = 0;

    buf[0] = 0x55;
    buf[1] = 0x55;
    buf[2] = motor->config.id;
    buf[3] = length;
    buf[4] = cmd;

    // У����ۼӴ� ID ��ʼ��buf[2], buf[3], buf[4] ...
    checksum_total = buf[2] + buf[3] + buf[4];

    for (uint8_t i = 0; i < param_len; i++) {
        buf[5 + i] = params[i];
        checksum_total += params[i];
    }

    // Checksum = ~(�ۼӺ͵ĵ�8λ)
    buf[5 + param_len] = (uint8_t)(~(checksum_total & 0xFF));

    USARTSend(motor->usart_instance, buf, total_len, USART_TRANSFER_BLOCKING);
}

void HEMotorMoveTimeWrite(HEMotor_Instance *motor, uint16_t pos, uint16_t time)
{
    if (motor == NULL || motor->ref.stop_flag == HE_STOP) return;

   
    uint8_t params[4];
    params[0] = pos & 0xFF;         // �Ƕȵ�λ
    params[1] = (pos >> 8) & 0xFF;  // �Ƕȸ�λ
    params[2] = time & 0xFF;        // ʱ���λ
    params[3] = (time >> 8) & 0xFF; // ʱ���λ

    HEMotorSendPacket(motor, SERVO_MOVE_TIME_WRITE, params, 4);
}

void HEMotorControl(void)
{
    static uint32_t last_send_tick = 0;
    uint32_t now = HAL_GetTick();

    // 限制发送频率为 50Hz (20ms 一次)，避免 1kHz 高频刷新导致舵机逻辑锁死或串口带宽饱和
    if (now - last_send_tick < 20) {
        return;
    }
    last_send_tick = now;

    for (int i = 0; i < he_idx; i++) {
        if (he_motor_instances[i] != NULL && he_motor_instances[i]->ref.stop_flag == HE_ENABLED) {
            HEMotorMoveTimeWrite(he_motor_instances[i], 
                                 he_motor_instances[i]->ref.position, 
                                 he_motor_instances[i]->ref.time);
        }
    }
}