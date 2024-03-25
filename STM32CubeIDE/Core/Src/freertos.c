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
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

union ISL_errors_t
{
	uint16_t all;
	struct{
		uint8_t OVF :1;		//Overvoltage Fault
		uint8_t OVLOF :1;	//Overvoltage Lockout Fault
		uint8_t UVF :1;		//Undervoltage Fault
		uint8_t UVLOF :1;	//Undervoltage Lockout Fault
		uint8_t DOTF :1;	//Discharge Over-Temperature Fault
		uint8_t DUTF :1;	//Discharge Under-Temperature Fault
		uint8_t COTF :1;	//Charge Over-Temperature Fault
		uint8_t CUTF :1;	//Charge Under-Temperature Fault

		uint8_t IOTF :1;	//Internal Over-Temperature Fault
		uint8_t COCF :1;	//Charge Overcurrent Fault
		uint8_t DOCF :1;	//Discharge Overcurrent Fault
		uint8_t DSCF :1;	//Discharge Short Circuit Fault
		uint8_t CELLF :1;	//Cell Fail fault
		uint8_t OPENF :1;	//Open-Wire Fault

		uint8_t CBOTF :1;	//Cell Balance Over-Temperature Fault
		uint8_t CBUTF :1;	//Cell Balance Under-Temperature Fault
	}bit;

	struct{
		uint8_t status0:8;
		uint8_t status1:6;
		uint8_t status3:2;
	}status;
};


union ISL_status_t
{
	uint8_t all;
	struct{
		uint8_t VEOC :1;	//Voltage End-of-Charge detection

		uint8_t CHING :1;	//Charging
		uint8_t DCHING :1;	//Discharging

		uint8_t CBOV :1;	//Cell Balance Overvoltage
		uint8_t CBUV :1;	//Cell Balance Undervoltage
		uint8_t IN_IDLE :1;	//IN_IDLE
		uint8_t IN_DOZE :1;	//IN_DOZE
		uint8_t IN_SLEEP :1;//IN_SLEEP
	}bit;

	struct{
		uint8_t status1 :1;
		uint8_t status2 :2;
		uint8_t status3 :5;
	}status;
};



typedef struct {
	union CONTROL0_REG Control0;
	union CONTROL1_REG Control1;
	union CONTROL2_REG Control2;
	union CONTROL3_REG Control3;


	/*
	 * 	Interrupt. This pin goes active low when there is an external MCU connected to the ISL94202 and MCU
		communication fails to send a slave byte within a watchdog timer period. This is a CMOS type output.
	 */
	uint8_t GPIO_INT;
	/*
	 * 	Shutdown. This output indicates that the ISL94202 detected a failure condition that would result in the DFET
		turning off. This could be undervoltage, over-temperature, under-temperature, etc. The SD pin also goes active
		if there is any charge overcurrent condition. This is an open-drain output.
	 */
	uint8_t GPIO_SD;
	/*
	 * 	Pack Shutdown. This pin is set high when any cell voltage reaches the OVLO threshold (OVLO flag).
		Optionally, PSD is also set if there is a voltage differential between any two cells that is greater than a specified
		limit (CELLF flag) or if there is an open-wire condition. This pin can be used with external circuitry for blowing a
		fuse in the pack or as an interrupt to an external MCU
	 */
	uint8_t GPIO_PSD;
	/*
	 * 	End-of-Charge. This output indicates that the ISL94202 detected a fully charged condition. This is defined by
		any cell voltage exceeding an EOC voltage (as defined by an EOC value in EEPROM).
	 */
	uint8_t GPIO_EOC;

	uint8_t GPIO_AUX1;
	uint8_t GPIO_AUX2;

	union ISL_errors_t Errors;
	union ISL_status_t Status;

	uint8_t CBFC;

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
int runer,runer2, CBFC, cnt ;

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

	  ISL94202_Init();

	  	myBms.Control0.CONTROL0 = 0;;
	  	myBms.Control0.CONTROL0_BIT.CG = 0x01;


	  	myBms.Control2.CONTROL2 = 0;
	  	myBms.Control2.CONTROL2_BIT.uC_FET = 1;
	  	myBms.Control2.CONTROL2_BIT.uC_CBAL = 1;
	  	myBms.Control2.CONTROL2_BIT.CBAL_ON = 1;

	  	myBms.Control3.CONTROL3_BIT.DOZE = 0;



	 	BMS_writeByte(CONTROL0_ADDR, myBms.Control0.CONTROL0);
	 	BMS_writeByte(CONTROL1_ADDR, myBms.Control1.CONTROL1);
	 	BMS_writeByte(CONTROL2_ADDR, myBms.Control2.CONTROL2);
	 	BMS_writeByte(CONTROL3_ADDR, myBms.Control3.CONTROL3);
	//Init_SDcard();




  /* Infinite loop */
	for(;;)
	{


	  if(runer >0){
	 	BMS_writeByte(CONTROL2_ADDR, myBms.Control2.CONTROL2);
	 	BMS_writeByte(CONTROL3_ADDR, myBms.Control3.CONTROL3);

	  }

	  if(runer2 >0){
	 	 BMS_writeByte(CBFC_ADDR, myBms.CBFC);


	  }


	  if (myBms.GPIO_EOC){
		  cnt++;
		  if (cnt >= 10){
			  printf(": BMS shutdown!!!\r\n");
			  myBms.Control3.CONTROL3_BIT.PDWN = 1;
			  BMS_writeByte(CONTROL3_ADDR, myBms.Control3.CONTROL3);
		  }

	  }else cnt = 0;

	  ISL94202_ReadAllMeasurements();

	  myBms.Errors.status.status0 = BMSOpRegs.STATUS0.STATUS0;
	  myBms.Errors.status.status1 = BMSOpRegs.STATUS1.STATUS1 & 0x3F;
	  myBms.Errors.status.status3 = BMSOpRegs.STATUS3.STATUS3 & 0x03;

	  myBms.Status.status.status1 = (BMSOpRegs.STATUS1.STATUS1>>7);
	  myBms.Status.status.status2 = (BMSOpRegs.STATUS2.STATUS2>>2) & 0x03;
	  myBms.Status.status.status3 = (BMSOpRegs.STATUS3.STATUS3>>2) & 0x1F;

	  myBms.GPIO_EOC 	= !HAL_GPIO_ReadPin(BMS_EOC_GPIO_Input_GPIO_Port, 	BMS_EOC_GPIO_Input_Pin);
	  myBms.GPIO_PSD 	= HAL_GPIO_ReadPin(BMS_EOC_PSD_Input_GPIO_Port, 	BMS_EOC_PSD_Input_Pin);
	  myBms.GPIO_INT	= HAL_GPIO_ReadPin(BMS_INT_GPIO_Input_GPIO_Port, 	BMS_INT_GPIO_Input_Pin);
	  myBms.GPIO_SD		= !HAL_GPIO_ReadPin(BMS_SD_GPIO_Input_GPIO_Port, 	BMS_SD_GPIO_Input_Pin);

	  myBms.GPIO_AUX2 =  myBms.GPIO_EOC;

	  HAL_GPIO_WritePin(AUX1_GPIO_Output_GPIO_Port, AUX1_GPIO_Output_Pin, myBms.GPIO_AUX1);
	  HAL_GPIO_WritePin(AUX2_GPIO_Output_GPIO_Port, AUX2_GPIO_Output_Pin, myBms.GPIO_AUX2);

	  r = 50 * (1.0f + sinf(counter));
	  counter++;
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, r);

	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

	  //timerTask();
      osDelay(500);


	memset(cpu.str_buf1, 0x00, sizeof(cpu.str_buf1));
	printf("\r\n");
	sprintf(cpu.str_buf1, "%d: Welcome from BMS_WIBAR\r\n", cpu.cnt++);
	printf((char *)cpu.str_buf1);
   // HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
   // printf("Welcome from BMS_WIBAR\r\n");
//    printf("------------------------\r\n");
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
