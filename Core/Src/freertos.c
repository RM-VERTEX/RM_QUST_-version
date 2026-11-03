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
osThreadId LEDHandle;
osThreadId IMUHandle;
osThreadId CHASSISHandle;
osThreadId DBUSHandle;
osThreadId BUZZERHandle;
osThreadId ADCHandle;
osThreadId GIMBALHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void led_task(void const * argument);
void imu_task(void const * argument);
void chassis_task(void const * argument);
void uart_task(void const * argument);
void buzzer_task(void const * argument);
void adc_task(void const * argument);
void gimbal_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
// [解析] ★ 这段是"给空闲任务静态分配内存"。
// [解析]   背景：FreeRTOS 里必然存在一个优先级为 0 的"空闲任务"(Idle Task)，
// [解析]   它的作用是"当所有任务都在延时/等待时，让 CPU 有事可做（并回收被删除任务的内存）"。
// [解析]   本工程在 FreeRTOSConfig.h 里同时打开了"静态分配"和"动态分配"
// [解析]   （configSUPPORT_STATIC_ALLOCATION = 1）。
// [解析]   一旦打开静态分配，FreeRTOS 就要求你【必须】自己提供空闲任务的内存
// [解析]   —— 也就是下面这两个数组；否则编译时会报缺少
// [解析]   vApplicationGetIdleTaskMemory 的链接错误。
// [解析]   好处是：空闲任务的内存不再从 heap 里分配，是编译期就确定的静态数组，
// [解析]   不会因为堆碎片或堆不够而创建失败。
static StaticTask_t xIdleTaskTCBBuffer;
// [解析] ↑ 空闲任务的"任务控制块"(TCB)，存放任务状态、优先级等信息。
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
// [解析] ↑ 空闲任务的栈。configMINIMAL_STACK_SIZE 在 FreeRTOSConfig.h 里是 128，
// [解析]   单位是"字"(StackType_t = 4 字节)，所以等于 512 字节。

// [解析] 这个函数由 FreeRTOS 内核自动调用（它在创建空闲任务时会来找你要内存）。
// [解析] 你只需要把上面那两个静态数组的地址"填进"输出的指针里就行。
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of LED */
  osThreadDef(LED, led_task, osPriorityLow, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of IMU */
  osThreadDef(IMU, imu_task, osPriorityBelowNormal, 0, 256);
  IMUHandle = osThreadCreate(osThread(IMU), NULL);

  /* definition and creation of CHASSIS */
  osThreadDef(CHASSIS, chassis_task, osPriorityHigh, 0, 256);
  CHASSISHandle = osThreadCreate(osThread(CHASSIS), NULL);

  /* definition and creation of DBUS */
  osThreadDef(DBUS, uart_task, osPriorityRealtime, 0, 256);
  DBUSHandle = osThreadCreate(osThread(DBUS), NULL);

  /* definition and creation of BUZZER */
  osThreadDef(BUZZER, buzzer_task, osPriorityAboveNormal, 0, 128);
  BUZZERHandle = osThreadCreate(osThread(BUZZER), NULL);

  /* definition and creation of ADC */
  osThreadDef(ADC, adc_task, osPriorityAboveNormal, 0, 128);
  ADCHandle = osThreadCreate(osThread(ADC), NULL);

  /* definition and creation of GIMBAL */
  osThreadDef(GIMBAL, gimbal_task, osPriorityHigh, 0, 256);
  GIMBALHandle = osThreadCreate(osThread(GIMBAL), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_led_task */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_task */
__weak void led_task(void const * argument)
{
  /* USER CODE BEGIN led_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_task */
}

/* USER CODE BEGIN Header_imu_task */
/**
* @brief Function implementing the IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_task */
__weak void imu_task(void const * argument)
{
  /* USER CODE BEGIN imu_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_task */
}

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the CHASSIS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void const * argument)
{
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_uart_task */
/**
* @brief Function implementing the DBUS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart_task */
__weak void uart_task(void const * argument)
{
  /* USER CODE BEGIN uart_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END uart_task */
}

/* USER CODE BEGIN Header_buzzer_task */
/**
* @brief Function implementing the BUZZER thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_buzzer_task */
__weak void buzzer_task(void const * argument)
{
  /* USER CODE BEGIN buzzer_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END buzzer_task */
}

/* USER CODE BEGIN Header_adc_task */
/**
* @brief Function implementing the ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_adc_task */
__weak void adc_task(void const * argument)
{
  /* USER CODE BEGIN adc_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END adc_task */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
* @brief Function implementing the GIMBAL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
