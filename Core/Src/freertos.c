/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Test.h"
#include "dji_motor.h"
#include "usb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for DJIMotor_Task */
osThreadId_t DJIMotor_TaskHandle;
const osThreadAttr_t DJIMotor_Task_attributes = {
  .name = "DJIMotor_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for usb_Task */
osThreadId_t usb_TaskHandle;
const osThreadAttr_t usb_Task_attributes = {
  .name = "usb_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ChassisControl(void *argument);
void DJIMotor(void *argument);
void Usb_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Chassis_Task */
  Chassis_TaskHandle = osThreadNew(ChassisControl, NULL, &Chassis_Task_attributes);

  /* creation of DJIMotor_Task */
  DJIMotor_TaskHandle = osThreadNew(DJIMotor, NULL, &DJIMotor_Task_attributes);

  /* creation of usb_Task */
  usb_TaskHandle = osThreadNew(Usb_Task, NULL, &usb_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ChassisControl */
/**
  * @brief  Function implementing the Chassis_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ChassisControl */
void ChassisControl(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN ChassisControl */
  /* Infinite loop */
  for(;;)
  {
		ChassisTask();
    osDelay(1);
  }
  /* USER CODE END ChassisControl */
}

/* USER CODE BEGIN Header_DJIMotor */
/**
* @brief Function implementing the DJIMotor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DJIMotor */
void DJIMotor(void *argument)
{
  /* USER CODE BEGIN DJIMotor */
  /* Infinite loop */
  for(;;)
  {
		DJIMotorControl(); 
    osDelay(1);
  }
  /* USER CODE END DJIMotor */
}

/* USER CODE BEGIN Header_Usb_Task */
/**
* @brief Function implementing the usb_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Usb_Task */
void Usb_Task(void *argument)
{
  /* USER CODE BEGIN Usb_Task */
  /* Infinite loop */
  for(;;)
  {
		USB_ProcessTask();
    osDelay(1);
  }
  /* USER CODE END Usb_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

