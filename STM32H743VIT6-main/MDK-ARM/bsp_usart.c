/**
 * @file bsp_usart.c
 * @author neozng,Bi Kaixiang (wexhicy@gmail.com)
 * @brief  ����bsp���ʵ��
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "bsp_usart.h"
#include "stdlib.h"
#include "string.h"

/* usart����ʵ��,����ע����usart��ģ����Ϣ�ᱻ���������� */
static uint8_t idx;
static USART_Instance *usart_instances[USART_DEVICE_MAX_NUM] = {NULL};

/**
 * @brief �������ڷ���,����ÿ��ʵ��ע��֮���Զ����ý���,��ǰʵ��ΪDMA����,������������IT��BLOCKING����
 *
 * @todo ���ڷ������ÿ��ʵ��ע��֮���Զ����ý���,��ǰʵ��ΪDMA����,������������IT��BLOCKING����
 *       ���ܻ�Ҫ���˺����޸�Ϊextern,ʹ��module���Կ��ƴ��ڵ���ͣ
 *
 * @param _instance instance owned by module,ģ��ӵ�еĴ���ʵ��
 */
void USARTServiceInit(USART_Instance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // �ر�dma half transfer�жϷ�ֹ���ν���HAL_UARTEx_RxEventCallback()
    // ����HAL���һ�����ʧ��,����DMA�������/������Լ�����IDLE�ж϶��ᴥ��HAL_UARTEx_RxEventCallback()
    // ����ֻϣ��������һ�ֺ͵��������,���ֱ�ӹر�DMA�봫���ж�
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

/**
 * @brief ע��һ������ʵ��,����һ������ʵ��ָ��
 *
 * @param init_config ���봮�ڳ�ʼ���ṹ��
 */
USART_Instance *USARTRegister(USART_Init_Config_s *init_config)
{
    USART_Instance *usart = (USART_Instance *)malloc(sizeof(USART_Instance));
    memset(usart, 0, sizeof(USART_Instance));

    usart->usart_handle    = init_config->usart_handle;
    usart->recv_buff_size  = init_config->recv_buff_size;
    usart->module_callback = init_config->module_callback;

    usart_instances[idx++] = usart;
    USARTServiceInit(usart);

    return usart;
}

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
void USARTSend(USART_Instance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE_e mode)
{
    switch (mode) {
        case USART_TRANSFER_BLOCKING:
            HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
            break;
        case USART_TRANSFER_IT:
            HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
            break;
        case USART_TRANSFER_DMA:
            HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
            break;
        default:
            break;
    }
}

/* ���ڷ���ʱ,gstate�ᱻ��ΪBUSY_TX */
uint8_t USARTIsReady(USART_Instance *_instance)
{
    if (_instance->usart_handle->gState & HAL_UART_STATE_BUSY_TX)
        return 0;
    else
        return 1;
}

/**
 * @brief ÿ��dma/idle�жϷ���ʱ��������ô˺���.����ÿ��uartʵ������ö�Ӧ�Ļص����н�һ���Ĵ���
 *        ����:�Ӿ�Э�����/ң��������/����ϵͳ����
 *
 * @note  ͨ��__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)�ر�dma half transfer�жϷ�ֹ���ν���HAL_UARTEx_RxEventCallback()
 *        ����HAL���һ�����ʧ��,����DMA�������/������Լ�����IDLE�ж϶��ᴥ��HAL_UARTEx_RxEventCallback()
 *        ����ֻϣ�����������ֱ�ӹر�DMA�봫���жϵ�һ�ֺ͵��������
 *
 * @param huart �����жϵĴ���
 * @param Size �˴ν��յ�����������,��ʱû��
 */
void USART_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i) {                  // find the instance which is being handled
        if (huart == usart_instances[i]->usart_handle) { // call the callback function if it is not NULL
            if (usart_instances[i]->module_callback != NULL) {
                usart_instances[i]->module_callback();
                memset(usart_instances[i]->recv_buff, 0, Size); // ���ս��������buffer,���ڱ䳤�����Ǳ�Ҫ��
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instances[i]->usart_handle, usart_instances[i]->recv_buff, usart_instances[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instances[i]->usart_handle->hdmarx, DMA_IT_HT);
            return; // break the loop
        }
    }
}

/**
 * @brief �����ڷ���/���ճ��ִ���ʱ,����ô˺���,��ʱ�������Ҫ���ľ���������������
 *
 * @note  ����Ĵ���:��żУ��/���/֡����
 *
 * @param huart ��������Ĵ���
 */
void USART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i) {
        if (huart == usart_instances[i]->usart_handle) {
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instances[i]->usart_handle, usart_instances[i]->recv_buff, usart_instances[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instances[i]->usart_handle->hdmarx, DMA_IT_HT);
            return;
        }
    }
}
