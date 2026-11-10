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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RcTask */
osThreadId_t RcTaskHandle;
const osThreadAttr_t RcTask_attributes = {
  .name = "RcTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RamdomTask */
osThreadId_t RamdomTaskHandle;
const osThreadAttr_t RamdomTask_attributes = {
  .name = "RamdomTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for JudgeTask */
osThreadId_t JudgeTaskHandle;
const osThreadAttr_t JudgeTask_attributes = {
  .name = "JudgeTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ErrorTask */
osThreadId_t ErrorTaskHandle;
const osThreadAttr_t ErrorTask_attributes = {
  .name = "ErrorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void OS_RcCallback(void *argument);
void OS_MotorCallback(void *argument);
void OS_RamdomCallback(void *argument);
void OS_JudgeCallback(void *argument);
void OS_ErrorCallback(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of RcTask */
  RcTaskHandle = osThreadNew(OS_RcCallback, NULL, &RcTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(OS_MotorCallback, NULL, &MotorTask_attributes);

  /* creation of RamdomTask */
  RamdomTaskHandle = osThreadNew(OS_RamdomCallback, NULL, &RamdomTask_attributes);

  /* creation of JudgeTask */
  JudgeTaskHandle = osThreadNew(OS_JudgeCallback, NULL, &JudgeTask_attributes);

  /* creation of ErrorTask */
  ErrorTaskHandle = osThreadNew(OS_ErrorCallback, NULL, &ErrorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_OS_RcCallback */
/**
* @brief Function implementing the RcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_RcCallback */
__weak void OS_RcCallback(void *argument)
{
  /* USER CODE BEGIN OS_RcCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_RcCallback */
}

/* USER CODE BEGIN Header_OS_MotorCallback */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_MotorCallback */
__weak void OS_MotorCallback(void *argument)
{
  /* USER CODE BEGIN OS_MotorCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_MotorCallback */
}

/* USER CODE BEGIN Header_OS_RamdomCallback */
/**
* @brief Function implementing the RamdomTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_RamdomCallback */
__weak void OS_RamdomCallback(void *argument)
{
  /* USER CODE BEGIN OS_RamdomCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_RamdomCallback */
}

/* USER CODE BEGIN Header_OS_JudgeCallback */
/**
* @brief Function implementing the JudgeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_JudgeCallback */
__weak void OS_JudgeCallback(void *argument)
{
  /* USER CODE BEGIN OS_JudgeCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_JudgeCallback */
}

/* USER CODE BEGIN Header_OS_ErrorCallback */
/**
* @brief Function implementing the ErrorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_ErrorCallback */
__weak void OS_ErrorCallback(void *argument)
{
  /* USER CODE BEGIN OS_ErrorCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_ErrorCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

