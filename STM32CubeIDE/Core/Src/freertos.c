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

#include "iwdg.h"
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
//#define BUT_WAKE  	0x0
//#define BUT_SLEEP  	0x1

#define ON  		0x1
#define OFF  		0x0

#define TIME_01s  	1U
#define TIME_05s  	5U
#define TIME_1s  	10U
#define TIME_2s  	20U
#define TIME_3s  	30U
#define TIME_5s  	50U
#define TIME_10s 	100U
#define TIME_12s 	120U
#define TIME_2min 	1200U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */




typedef enum
{
	BUT_WAKE,
	BUT_SLEEP
}eBut_state;

typedef enum
{
	LED_ON_BOARD,
	LED_ERROR,
	LED_OK,
	LED_90,
	LED_30
}eLed_state;


typedef struct {
	float out;
	float Kahan;
} adc_t;

static float Ceil_3(float liczba)
{
	float wynik = liczba*1000;
	int a = (int)wynik;

	wynik =(float)a/1000;
	return wynik;
}


float Filter1_calc(float input, adc_t* add, float Ts_Ti)
{
	float integrator_last = add->out;
	float y = Ts_Ti * (input - integrator_last) - add->Kahan;
	add->out = integrator_last + y;
	add->Kahan = (add->out - integrator_last) - y;
	return add->out;
}

adc_t adc[14];
static float adc_Ts_Ti = 0.075;


static int cnt2;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Shutdown_all(void){
	f_close(&MyFile);
	FATFS_UnLinkDriver(SDPath);


	printf(": BMS force shutdown!!!\r\n");
	myBms.Control3.CONTROL3_BIT.PDWN = 1;
	BMS_writeByte(CONTROL3_ADDR, myBms.Control3.CONTROL3);
}



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
	// For LED3
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	bms.init = 1;
	//cpu.SDCardSave = 1;


	//write config ISL
	ISL94202_Init();

	//Control 0
	myBms.Control0.CONTROL0 = 0;;
	myBms.Control0.CONTROL0_BIT.CG = 0x01;	// The Current Gain setting to x5
	BMS_writeByte(CONTROL0_ADDR, myBms.Control0.CONTROL0);
	//Control 1
	BMS_writeByte(CONTROL1_ADDR, myBms.Control1.CONTROL1);
	//Control 2
	myBms.Control2.CONTROL2 = 0;
	myBms.Control2.CONTROL2_BIT.uC_FET = 1;
	myBms.Control2.CONTROL2_BIT.uC_CBAL = 0;
	myBms.Control2.CONTROL2_BIT.CBAL_ON = 1;
	BMS_writeByte(CONTROL2_ADDR, myBms.Control2.CONTROL2);
	//Control 3
	myBms.Control3.CONTROL3_BIT.DOZE = 0;
	BMS_writeByte(CONTROL3_ADDR, myBms.Control3.CONTROL3);



  /* Infinite loop */
	for(;;){
		ISL94202_ReadCellVoltages();
		BatteryPack = CheckBatteryStatus();
		ISL94202_ReadPackCurrent();
		ISL94202_ReadTemperatures();
		ISL94202_ReadPackVoltage();
		ISL94202_MinMaxVoltage();
		ISL94202_ReadVRGO();
		ISL94202_ReadStatusRegisters();
		ISL94202_ReadControlRegisters();



		myBms.Errors.bit.OVF	= BMSOpRegs.STATUS0.STATUS0_BIT.OVF;
		myBms.Errors.bit.OVLOF	= BMSOpRegs.STATUS0.STATUS0_BIT.OVLOF;
		myBms.Errors.bit.UVF	= BMSOpRegs.STATUS0.STATUS0_BIT.UVF;
		myBms.Errors.bit.UVLOF	= BMSOpRegs.STATUS0.STATUS0_BIT.UVLOF;
		myBms.Errors.bit.DOTF	= BMSOpRegs.STATUS0.STATUS0_BIT.DOTF;
		myBms.Errors.bit.DUTF	= BMSOpRegs.STATUS0.STATUS0_BIT.DUTF;
		myBms.Errors.bit.COTF	= BMSOpRegs.STATUS0.STATUS0_BIT.COTF;
		myBms.Errors.bit.CUTF	= BMSOpRegs.STATUS0.STATUS0_BIT.CUTF;

		myBms.Errors.bit.IOTF	= BMSOpRegs.STATUS1.STATUS1_BIT.IOTF;
		myBms.Errors.bit.COCF	= BMSOpRegs.STATUS1.STATUS1_BIT.COCF;
		myBms.Errors.bit.DOCF	= BMSOpRegs.STATUS1.STATUS1_BIT.DOCF;
		myBms.Errors.bit.DSCF	= BMSOpRegs.STATUS1.STATUS1_BIT.DSCF;
		myBms.Errors.bit.CELLF	= BMSOpRegs.STATUS1.STATUS1_BIT.CELLF;
		myBms.Errors.bit.OPENF	= BMSOpRegs.STATUS1.STATUS1_BIT.OPENF;

		myBms.Errors.bit.CBOT	= BMSOpRegs.STATUS3.STATUS3_BIT.CBOT;
		myBms.Errors.bit.CBUT	= BMSOpRegs.STATUS3.STATUS3_BIT.CBUT;




		//filter adc measurements
		for(uint8_t i=0;i<8;i++){
			bms.CellVoltages[i] = Ceil_3(Filter1_calc(BMSMeasurements.CellVoltages[i],&adc[i],adc_Ts_Ti));
		}
		bms.Vcellmin 	= Ceil_3(Filter1_calc(BMSMeasurements.Vcellmin,&adc[8],adc_Ts_Ti));
		bms.Vcellmax 	= Ceil_3(Filter1_calc(BMSMeasurements.Vcellmax,&adc[9],adc_Ts_Ti));
		bms.PackCurrent = Ceil_3(Filter1_calc(BMSMeasurements.PackCurrent,&adc[10],adc_Ts_Ti*10));
		bms.Vpack 		= Ceil_3(Filter1_calc(BMSMeasurements.Vpack,&adc[11],adc_Ts_Ti));
		bms.Temp1 		= Ceil_3(Filter1_calc(BMSMeasurements.Temp1,&adc[12],adc_Ts_Ti));
		bms.Temp2 		= Ceil_3(Filter1_calc(BMSMeasurements.Temp2,&adc[13],adc_Ts_Ti));

		if(bms.init){
			bms.init = 0;
			bms.state = STATE_DISCARGE;
			//Settings
			bms.set.CellDiffvalue_V 		= 0.8;
			bms.set.MaxTemperature_degree 	= 60.0;
			bms.set.MinTemperature_degree 	= 0.0;
			bms.set.PackVoltageMax_V 		= 29.5;
			bms.set.PackChargeVoltage_V 	= 29.3;
			bms.set.PackVoltageLed30_V		= 22.5;
			bms.set.PackDischargeVoltage_V 	= 20.5;
			bms.set.PackVoltageMin_V 		= 19.0;
			bms.set.CellDiffvalue_cnt		= 200;
			bms.set.CelVoltageMax_V			= 4.25;
			bms.set.CelVoltageMin_V			= 2.4;
		}


		//Main program
		if(bms.cntStart > TIME_3s){


			if(bms.Vcellmax > bms.set.CelVoltageMax_V){
				bms.alarms_cnt[5]++;
			}else bms.alarms_cnt[5] = 0;
			if ( bms.alarms_cnt[5] > TIME_5s) bms.alarms.bit.CellOverVoltage = 1;


			//Calculations
			bms.cal.CellDiff_V = bms.Vcellmax - bms.Vcellmin;
			if (bms.cal.CellDiff_V > bms.set.CellDiffvalue_V) 	bms.cal.CellDiff_cnt++;
			else												bms.cal.CellDiff_cnt = 0;
			//Buttons
			//bms.gpio.ButtonState[BUT_WAKE] =  1;
			bms.gpio.ButtonState[BUT_WAKE] =  HAL_GPIO_ReadPin(BUT_GPIO_input_GPIO_Port, BUT_GPIO_input_Pin);
	//TODO:don't use button
			bms.gpio.ButtonState[BUT_SLEEP] =  0;


			//ALARMS
			if((bms.Temp1 > bms.set.MaxTemperature_degree ||
					bms.Temp2 > bms.set.MaxTemperature_degree)){
				bms.alarms_cnt[0]++;
			}else bms.alarms_cnt[0] = 0;
			if ( bms.alarms_cnt[0] > TIME_5s) bms.alarms.bit.OverTemperature = 1;

			if((bms.Temp1 < bms.set.MinTemperature_degree ||
					bms.Temp2 < bms.set.MinTemperature_degree)){
				bms.alarms_cnt[1]++;
			}else bms.alarms_cnt[1] = 0;
			if ( bms.alarms_cnt[1] > TIME_5s) bms.alarms.bit.UnderTemperature = 1;

			if(bms.Vpack > bms.set.PackVoltageMax_V){
				bms.alarms_cnt[2]++;
			}else bms.alarms_cnt[2] = 0;
			if ( bms.alarms_cnt[2] > TIME_5s) bms.alarms.bit.PackOverVoltage = 1;

			if(bms.Vpack < bms.set.PackVoltageMin_V){
				bms.alarms_cnt[3]++;
			}else bms.alarms_cnt[3] = 0;
			if ( bms.alarms_cnt[3] > TIME_5s) bms.alarms.bit.PackUnderVoltage = 1;

			if(bms.cal.CellDiff_cnt > bms.set.CellDiffvalue_cnt){
				bms.alarms_cnt[3]++;
			}else bms.alarms_cnt[3] = 0;
			if ( bms.alarms_cnt[3] > TIME_5s) bms.alarms.bit.CellsDiferenceVoltage = 1;

			if(bms.Vcellmin < bms.set.CelVoltageMin_V){
				bms.alarms_cnt[4]++;
			}else bms.alarms_cnt[4] = 0;
			if ( bms.alarms_cnt[4] > TIME_5s) bms.alarms.bit.CellUnderVoltage = 1;

			if(bms.Vcellmax > bms.set.CelVoltageMax_V){
				bms.alarms_cnt[5]++;
			}else bms.alarms_cnt[5] = 0;
			if ( bms.alarms_cnt[5] > TIME_5s) bms.alarms.bit.CellOverVoltage = 1;

			//if (bms.alarms.bit.ISL_OVLO) 	BMSOpRegs.STATUS0.STATUS0_BIT.OVLOF = 1;
			//if (bms.alarms.bit.ISL_OV) 	BMSOpRegs.STATUS0.STATUS0_BIT.OVF	= 1;
			//if (bms.alarms.bit.ISL_UV) 	BMSOpRegs.STATUS0.STATUS0_BIT.UVF	= 1;
			//if (bms.alarms.bit.ISL_UVLO)	BMSOpRegs.STATUS0.STATUS0_BIT.UVLOF = 1;

			switch(bms.state)
			{
			   case STATE_IDLE:
				   if(bms.gpio.ButtonState[BUT_WAKE]) bms.ButBlocked = 0;
				   if (!bms.ButBlocked){
					   if (bms.gpio.ButtonState[BUT_WAKE])	bms.cntSleep = bms.cntSleep+1;
					   else									bms.cntSleep = 0;
				   }

				   if (bms.cntSleep >= TIME_1s) {
					   bms.alarms.all = 0;
					   bms.cntSleep = 0;
					   bms.state = STATE_DISCARGE;
				   }
				   //bms.state = STATE_DISCARGE;
				   break;

			   case STATE_CHARGE:
				   //check errors
				   if(bms.alarms.all){
					   bms.state = STATE_ERROR;
					   break;
				   }

				   //Idle state, absorbing  charge
				   //if(bms.PackCurrent == 0){
					//   if(bms.cnt[0] > TIME_2min){
					//	   bms.AbsorbingCharge = 1;
					//   }else bms.cnt[0]++;
				  // }else{
					//   bms.cnt[0] = 0;
					//   bms.AbsorbingCharge = 0;
				  /// }

				   //change state to discharging
				   if(bms.PackCurrent < 0){
					   if(bms.cnt[1]++ > TIME_05s){
						   bms.cnt[1] = 0;
						   bms.state = STATE_DISCARGE;
						   break;
					   }
				   }else{
					   bms.cnt[1] = 0;
				   }

				   //charge recognition
				   if(bms.Vpack > bms.set.PackChargeVoltage_V){
					   if(bms.cnt[2]++ > TIME_5s){
						   bms.cnt[2] = 0;
						   //bms.state = STATE_IDLE;
						   //break;
					   }
				   }else{
					   bms.cnt[2] = 0;
				   }

				   //force end of discharging
				   if (bms.gpio.ButtonState[BUT_WAKE])	bms.cntSleep = bms.cntSleep+1;
				   else									bms.cntSleep = 0;

				   if (bms.cntSleep >= TIME_1s) {
					   bms.ButBlocked = 1;
					   bms.cntSleep = 0;
					   Shutdown_all();
				   }

				   break;


			   case STATE_DISCARGE:
				   //check errors
				   if(bms.alarms.all){
					   bms.state = STATE_ERROR;
					   break;
				   }

				   //turning off charging
				   if(bms.PackCurrent > 0){
					   if(bms.cnt[0]++ > TIME_10s){
						   bms.cnt[0] = 0;
						   bms.state = STATE_CHARGE;
						   break;
					   }
				   }else{
					   bms.cnt[0] = 0;
				   }


					if(bms.Vpack < bms.set.PackDischargeVoltage_V){
					   if(bms.cnt[2]++ > TIME_12s){
						   bms.cnt[2] = 0;
						   bms.state = STATE_ERROR;
					   }
					}else{
					   bms.cnt[2] = 0;
					}

				   //force end of charging
				   if (bms.gpio.ButtonState[BUT_WAKE])	bms.cntSleep = bms.cntSleep+1;
				   else									bms.cntSleep = 0;

				   if (bms.cntSleep >= TIME_1s) {
					   bms.ButBlocked = 1;
					   bms.cntSleep = 0;
					   Shutdown_all();
				   }

				   break;


			   case STATE_ERROR:
				   //Auto sleep
				   bms.cntErrorSleep++;
				   if (bms.cntErrorSleep >= TIME_2min) {
					   bms.cntErrorSleep = 0;
					   Shutdown_all();
				   }

				   //force end of charging
				   if (bms.gpio.ButtonState[BUT_WAKE])	bms.cntSleep = bms.cntSleep+1;
				   else									bms.cntSleep = 0;

				   if (bms.cntSleep >= TIME_1s) {
					   bms.ButBlocked = 1;
					   bms.cntSleep = 0;
					   bms.state = STATE_IDLE;
				   }
				   break;

			   default:
				   bms.state = STATE_IDLE;
				   break;
			}


			if(bms.Vpack > bms.set.PackChargeVoltage_V-0.1) bms.AbsorbingCharge = 1;
			else											bms.AbsorbingCharge = 0;


			//LED OK
			if(bms.gpio.led_cnt[0] == TIME_2s) 	bms.gpio.led_cnt[0] = 0;
			else								bms.gpio.led_cnt[0]++;
			if(bms.gpio.led_cnt[1] == TIME_05s) bms.gpio.led_cnt[1] = 0;
			else								bms.gpio.led_cnt[1]++;

			if(bms.state == STATE_ERROR)				bms.gpio.LedState[LED_OK] 	= OFF;
			else if(bms.state == STATE_CHARGE
					&& bms.AbsorbingCharge == 0
					&& bms.gpio.led_cnt[1] >= TIME_05s-3)bms.gpio.LedState[LED_OK] 	= ON;
			else if(bms.state == STATE_CHARGE
					&& bms.AbsorbingCharge == 1)		bms.gpio.LedState[LED_OK] 	= ON;
			else if(bms.state != STATE_CHARGE
					&& bms.gpio.led_cnt[0] >=TIME_2s-1)	bms.gpio.LedState[LED_OK] 	= ON;
			else										bms.gpio.LedState[LED_OK] 	= OFF;

			//LED 30
			if(BMSMeasurements.Vpack < bms.set.PackVoltageLed30_V) 	bms.gpio.LedState[LED_30] = ON;
			else													bms.gpio.LedState[LED_30] = OFF;


			//LED 90
			if(bms.cal.CellDiff_V >0.3){
				if(bms.gpio.led_cnt[0] >=TIME_2s-1)	bms.gpio.LedState[LED_90] 	= !bms.gpio.ConnectorsState[0];
				else								bms.gpio.LedState[LED_90] 	= bms.gpio.ConnectorsState[0];
			}
			else									bms.gpio.LedState[LED_90] 	= bms.gpio.ConnectorsState[0];



			//LED ERROR
			if(bms.cal.CellDiff_V >0.8){
				if(bms.gpio.led_cnt[0] >=TIME_2s-1)	bms.gpio.LedState[LED_ERROR] = OFF;
				else								bms.gpio.LedState[LED_ERROR] = ON;
			}
			else if (bms.state == STATE_ERROR)		bms.gpio.LedState[LED_ERROR] = ON;
			else									bms.gpio.LedState[LED_ERROR] = OFF;

			//LED ON_BOARD
			bms.gpio.LedState[LED_ON_BOARD] = bms.gpio.LedState[LED_OK];



			if (bms.state == STATE_CHARGE || bms.state == STATE_DISCARGE)	bms.gpio.ConnectorsState[0]	= ON;
			else															bms.gpio.ConnectorsState[0]	= OFF;
			myBms.GPIO_AUX2 =  bms.gpio.ConnectorsState[0];

			// ConnectorsState 1
			bms.gpio.ConnectorsState[1]		= OFF;


			HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, 		!bms.gpio.LedState[LED_ERROR]);
			HAL_GPIO_WritePin(LED_NEED_GPIO_Port, LED_NEED_Pin, 		!bms.gpio.LedState[LED_30]);
			HAL_GPIO_WritePin(LED_CHARGE_GPIO_Port, LED_CHARGE_Pin,		!bms.gpio.LedState[LED_90]);
			HAL_GPIO_WritePin(LED_OK_GPIO_Port, LED_OK_Pin, 			!bms.gpio.LedState[LED_OK]);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 				!bms.gpio.LedState[LED_ON_BOARD]);
			//HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, 				bms.gpio.ConnectorsState[0]);
			//HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, 				bms.gpio.ConnectorsState[1]);
			HAL_GPIO_WritePin(AUX1_GPIO_Output_GPIO_Port, AUX1_GPIO_Output_Pin, myBms.GPIO_AUX1);
			HAL_GPIO_WritePin(AUX2_GPIO_Output_GPIO_Port, AUX2_GPIO_Output_Pin, myBms.GPIO_AUX2);



			myBms.Errors.status.status0 = BMSOpRegs.STATUS0.STATUS0;
			myBms.Errors.status.status1 = BMSOpRegs.STATUS1.STATUS1 & 0x3F;
			myBms.Errors.status.status3 = BMSOpRegs.STATUS3.STATUS3 & 0x03;

			myBms.Status.status.status1 = (BMSOpRegs.STATUS1.STATUS1>>7);
			myBms.Status.status.status2 = (BMSOpRegs.STATUS2.STATUS2>>2) & 0x03;
			myBms.Status.status.status3 = (BMSOpRegs.STATUS3.STATUS3>>2) & 0x1F;

			myBms.GPIO_EOC 	= !HAL_GPIO_ReadPin(BMS_EOC_GPIO_Input_GPIO_Port, 	BMS_EOC_GPIO_Input_Pin);
			myBms.GPIO_PSD 	= HAL_GPIO_ReadPin(BMS_EOC_PSD_Input_GPIO_Port, 	BMS_EOC_PSD_Input_Pin);
			myBms.GPIO_INT	= HAL_GPIO_ReadPin(BMS_INT_GPIO_Input_GPIO_Port, 	BMS_INT_GPIO_Input_Pin);
			myBms.GPIO_SD	= !HAL_GPIO_ReadPin(BMS_SD_GPIO_Input_GPIO_Port, 	BMS_SD_GPIO_Input_Pin);



			// Force manual balancing
			if (!myBms.Status.bit.DCHING && !myBms.Status.bit.CHING){
				myBms.ForceBalancing = 0;
			}else{
				myBms.ForceBalancing = 0;
			}

			if (myBms.ForceBalancing){
				//only one
				if (!BMSOpRegs.CONTROL2.CONTROL2_BIT.uC_CBAL){
					myBms.Control2.CONTROL2_BIT.uC_CBAL = 1;
					BMS_writeByte(CONTROL2_ADDR, myBms.Control2.CONTROL2);
				}
				//myBms.CBFC = 0xff;
				BMS_writeByte(CBFC_ADDR, myBms.CBFC);
			}else{
				//only one
				if (BMSOpRegs.CONTROL2.CONTROL2_BIT.uC_CBAL){
					myBms.Control2.CONTROL2_BIT.uC_CBAL = 0;
					BMS_writeByte(CONTROL2_ADDR, myBms.Control2.CONTROL2);
				}
			}



		  if (myBms.GPIO_SD && myBms.Status.bit.DCHING){
			  if (cnt2++ >= TIME_10s){
				  cnt2 = 0;
				  Shutdown_all();
			  }
		  }else{
			  cnt2 = 0;
		  }


		  //LED 3
		  if (myBms.GPIO_SD)__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 90);
		  else __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 60);


		}else{
			bms.cntStart++;
		}



		if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK) 	myBms.cntWork[0]++;//(1/32000)*256*2500, so max time is 20 s
		else 									myBms.cntWork[1]++;


		osDelay(100);
	//	timerTask();

	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
