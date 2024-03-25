/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "math.h"
#include "stdio.h"

#include "mydevice.h"
#include "ISL94202.h"
#include "BMS.h"
//#include "FSM.h"
#include "TCA9554.h"
//#include "onewire.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

typedef struct {
	uint8_t GPIO_INT;
	uint8_t GPIO_SD;
	uint8_t GPIO_PSD;
	uint8_t GPIO_EOC;

	uint8_t GPIO_AUX1;
	uint8_t GPIO_AUX2;
} BMS_t;
BMS_t myBms;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	uint8_t counter;
	float r;

	//  ISL94202_Init();

	Init_SDcard();





  /* Infinite loop */
  for(;;)
  {
	  //ISL94202_ReadAllMeasurements();


	  myBms.GPIO_EOC = HAL_GPIO_ReadPin(BMS_EOC_GPIO_Input_GPIO_Port, 	BMS_EOC_GPIO_Input_Pin);
	  myBms.GPIO_SD  = HAL_GPIO_ReadPin(BMS_SD_GPIO_Input_GPIO_Port, 	BMS_SD_GPIO_Input_Pin);
	  myBms.GPIO_PSD = HAL_GPIO_ReadPin(BMS_EOC_PSD_Input_GPIO_Port, 	BMS_EOC_PSD_Input_Pin);
	  myBms.GPIO_INT = HAL_GPIO_ReadPin(BMS_INT_GPIO_Input_GPIO_Port, 	BMS_INT_GPIO_Input_Pin);


	  HAL_GPIO_WritePin(AUX1_GPIO_Output_GPIO_Port, AUX1_GPIO_Output_Pin, myBms.GPIO_AUX1);
	  HAL_GPIO_WritePin(AUX2_GPIO_Output_GPIO_Port, AUX2_GPIO_Output_Pin, myBms.GPIO_AUX2);

	  r = 50 * (1.0f + sinf(counter));
	  counter++;
	//HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, r);

	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

	 timerTask();
    osDelay(500);



   // HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
   // printf("Welcome from BMS_WIBAR\r\n");
//    printf("------------------------\r\n");
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
