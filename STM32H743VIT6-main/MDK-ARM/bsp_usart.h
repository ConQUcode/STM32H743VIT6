/**
 * @file bsp_uart.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   UART
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef BSP_USART_H
#define BSP_USART_H

#include "stdlib.h"
#include "usart.h"

#define USART_DEVICE_MAX_NUM 3    // ֧�ֵ����USART�豸����
#define USART_RXBUFF_LIMIT   255u // ���Э����Ҫ�����buff,���޸�����

// ģ��ص�����,���ڽ���Э��
typedef void (*usart_module_callback)();

/* ����ģʽö�� */
typedef enum {
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE_e;

// ����ʵ���ṹ��,ÿ��module��Ҫ����һ��ʵ��.
// ���ڴ����Ƕ�ռ�ĵ�Ե�ͨ��,���Բ���Ҫ���Ƕ��moduleͬʱʹ��һ�����ڵ����,��˲��ü���id;��ȻҲ����ѡ�����,������bsp����Է��ʵ�module��������Ϣ
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // Ԥ�ȶ�������buff��С,���̫С���޸�USART_RXBUFF_LIMIT
    uint8_t recv_buff_size;                // ģ�����һ�����ݵĴ�С
    UART_HandleTypeDef *usart_handle;      // ʵ����Ӧ��usart_handle
    usart_module_callback module_callback; // �����յ������ݵĻص�����
} USART_Instance;

// ���ڳ�ʼ�����ýṹ��
typedef struct
{
    uint8_t recv_buff_size;                // ģ�����һ�����ݵĴ�С
    UART_HandleTypeDef *usart_handle;      // ʵ����Ӧ��usart_handle
    usart_module_callback module_callback; // �����յ������ݵĻص�����
} USART_Init_Config_s;

/**
 * @brief �������ڷ���,����ÿ��ʵ��ע��֮���Զ����ý���,��ǰʵ��ΪDMA����,������������IT��BLOCKING����
 *
 * @todo ���ڷ������ÿ��ʵ��ע��֮���Զ����ý���,��ǰʵ��ΪDMA����,������������IT��BLOCKING����
 *       ���ܻ�Ҫ���˺����޸�Ϊextern,ʹ��module���Կ��ƴ��ڵ���ͣ
 *
 * @param _instance instance owned by module,ģ��ӵ�еĴ���ʵ��
 */
void USARTServiceInit(USART_Instance *_instance);

/**
 * @brief ע��һ������ʵ��,����һ������ʵ��ָ��
 *
 * @param init_config ���봮�ڳ�ʼ���ṹ��
 */
USART_Instance *USARTRegister(USART_Init_Config_s *init_config);

/**
 * @brief ͨ�����øú������Է���һ֡����,��Ҫ����һ��usartʵ��,����buff�Լ���һ֡�ĳ���
 * @note �ڶ�ʱ�����������ô˽ӿ�,������IT/DMA�ᵼ����һ�εķ���δ��ɶ��µķ���ȡ��.
 * @note ��ϣ������ʹ��DMA/IT���з���,�����USARTIsReady()ʹ��,������Ϊ���moduleʵ��һ�����Ͷ��к�����.
 * @todo �Ƿ���ΪUSARTInstance���ӷ��Ͷ����Խ�����������?
 *
 * @param _instance ����ʵ��
 * @param send_buf ���������ݵ�buffer
 * @param send_size how many bytes to send
 */
void USARTSend(USART_Instance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE_e mode);

/**
 * @brief �жϴ����Ƿ�׼����,�����������첽��IT/DMA����
 *
 * @param _instance Ҫ�жϵĴ���ʵ��
 * @return uint8_t ready 1, busy 0
 */
uint8_t USARTIsReady(USART_Instance *_instance);

void USART_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);
void USART_ErrorCallback(UART_HandleTypeDef *huart);

#endif // BSP_USART_H