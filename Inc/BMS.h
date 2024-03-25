/*
 * BMS.h
 *
 *  Created on: Jun 6, 2020
 *      Author: bnowa
 */

#ifndef BMS_H_
#define BMS_H_


#include "stdint.h"
#include "main.h"
#include "ISL94202.h"
//#include "onewire.h"
#include "main.h"
#include "TCA9554.h"
//#include "ds18b20.h"

#define ENABLE_CAN HAL_GPIO_WritePin(EN_CAN_GPIO_Port, EN_CAN_Pin, GPIO_PIN_RESET);

#define DISABLE_CAN HAL_GPIO_WritePin(EN_CAN_GPIO_Port, EN_CAN_Pin, GPIO_PIN_SET);


#define NEED_CHRG HAL_GPIO_WritePin(LED_CHRG_Port, LED_CHRG_Pin, GPIO_PIN_RESET);
//extern volatile OneWire_t OneWireBus;
extern uint32_t idlecnt;
//**********************************





#define CELLNUM 7

#define V_THRESHOLD_EHES_CONTROLLER_UP 2.7

#define V_THRESHOLD_EHES_CONTROLLER_DOWN 2.5

#define V_THRESHOLD_AUX_SUPPLY_UP 3.0

#define V_THRESHOLD_AUX_SUPPLY_DOWN 2.8



#define NOM_CAPACITY 6.0 // mAh
#define TS 0.1 // s
#define SOC_CONSTANT (TS / 3600.0)/ 6.0
//**********************************
#define 	SOCp1   -0.7257
#define     SOCp2    9.578
#define     SOCp3   -46.41
#define     SOCp4    98.29
#define     SOCp5 	-76.99



#define hExtADC hadc1



extern volatile uint8_t CAN_FET_Control, AUX_Control;


//*************************** MCP9700 constants
#define Tc 0.01
#define V0 0.5

//*************************** Divider measurements constants
#define VBAT_DIV 11.0
#define VIN_DIV 11.0


#define STATUS_LED_ON HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
#define STATUS_LED_OFF HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);




	union extMeas{
	struct extMeasStruct {
		uint16_t VIN_CHRG;
		uint16_t VBAT;
		uint16_t BAT_CURR;
		uint16_t MCP9700A;
		uint16_t Vref;

	}extMeasurementsStruct;

	uint16_t MeasurementArray[5];
	} ;

	union fExtMeas{
	struct fExtMeasStruct {
		float VIN_CHRG;
		float VBAT;
		float BAT_CURR;
		float MCP9700A;

	}fExtMeasurementsStruct;

	float fMeasurementArray[4];
	};



//********************************************


//	uint16_t VREFINT_CAL;


typedef enum BatteryState {Charging,Discharging,Idle}BatteryState;
extern BatteryState BatteryPack;
extern volatile double BatterySOC;
extern uint8_t heaterEnable;
//********************************************
void CheckForFaults(void);
void StartExternalMeasurements(void);
void ConvertExternalMeasurements(void);

void EnableCharger(void);
void DisableCharger(void);

void BMSWakeUp(void);


BatteryState  CheckBatteryStatus(void);
void UpdateLED(BatteryState BatteryPack);


void BMSCheckTemp(void);


void BMSCheckSOC(void);
void BMSSaveSOCValue(void);

void BackupRegisterInit(void);
void BackupRegWrite(uint8_t regnum,uint32_t data);
uint32_t BackupRegRead(uint32_t regnum);




#endif /* BMS_H_ */
