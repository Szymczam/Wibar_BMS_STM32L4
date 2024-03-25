/*
 * TCA9554.h
 *
 *  Created on: Feb 8, 2021
 *      Author: bnowa
 */

#ifndef TCA9554_H_
#define TCA9554_H_

#include "main.h"
#include "stdint.h"
//#include "ds18b20.h"
#include "math.h"

	#define TCA9554_I2C_PORT hi2c1
	extern I2C_HandleTypeDef TCA9554_I2C_PORT;
	#define TCA9544_I2C_ADDR 0x20

	#define HEATER_TIMER htim2
	extern TIM_HandleTypeDef htim2;


//extern volatile OneWire_t OneWireBus;

#define HEAT 0
#define COOL 1



extern float* T1_P;
extern float* T2_P;
extern float* T3_P;
extern float* T4_P;

extern float* T1_WYM;
extern float* T2_WYM;
extern float* T3_WYM;

extern float* FET_Temp;
extern float* H_BRIDGE_Temp;

typedef struct{
	float* T1;
	float* T2;
	float* T3;
	float* T4;

	float Tmax;
	float Tmin;
	uint8_t DIR;
	uint8_t DIS;
	uint8_t overheat;
	uint8_t underheat;
	float duty;

}
heater_t;

extern volatile heater_t h[4];
extern heater_t h1,h2,h3,h4;

typedef union
{
struct{
	uint8_t P0:1;
	uint8_t P1:1;
	uint8_t P2:1;
	uint8_t P3:1;
	uint8_t P4:1;
	uint8_t P5:1;
	uint8_t P6:1;
	uint8_t P7:1;
}outputBits_t;

uint8_t outputBuffer;
}
outputPort_t;

extern volatile outputPort_t output;
extern uint8_t heaterControl;
void TCA9544_Config(void);
void TCA9544_SetOutput(uint8_t portState);


void initHeaters(void);
void updateHeaters(void);
#endif /* TCA9554_H_ */
