
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYDEVICE_H
#define __MYDEVICE_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h"
#include "ISL94202.h"

extern  SD_HandleTypeDef hsd1;

#define BSP_SD_CardInfo HAL_SD_CardInfoTypeDef

#define MSD_OK                        	((uint8_t)0x00)
#define MSD_ERROR                     	((uint8_t)0x01)
#define MSD_ERROR_SD_NOT_PRESENT      	((uint8_t)0x02)
#define SD_TRANSFER_OK                	((uint8_t)0x00)
#define SD_TRANSFER_BUSY              	((uint8_t)0x01)
#define SD_PRESENT               		((uint8_t)0x01)
#define SD_NOT_PRESENT           		((uint8_t)0x00)
#define SD_DATATIMEOUT           		((uint32_t)100000000)



typedef struct{
	float CellDiff_V;
	uint16_t CellDiff_cnt;
} BMS_calculations;

typedef struct{
	uint8_t ButtonState[2];
	uint8_t LedState[5];
	uint8_t ConnectorsState[2];
	uint16_t led_cnt[2];
} BMS_gpio;

typedef struct{
	float CellDiffvalue_V;
	uint16_t CellDiffvalue_cnt;
	float MaxTemperature_degree;
	float MinTemperature_degree;
	float PackVoltageMax_V;
	float PackVoltageMin_V;
	float PackChargeVoltage_V;
	float PackDischargeVoltage_V;
	float PackVoltageLed30_V;
	float CelVoltageMax_V;
	float CelVoltageMin_V;
} BMS_settings;

typedef enum
{
	STATE_IDLE,
	STATE_CHARGE,
	STATE_DISCARGE,
	STATE_ERROR
} eState;

union BMS_ALARMS
{
	uint16_t all;
	struct{
		uint16_t PackOverVoltage : 1;
		uint16_t PackUnderVoltage : 1;
		uint16_t OverTemperature : 1;
		uint16_t UnderTemperature : 1;
		uint16_t CellsDiferenceVoltage : 1;

		uint16_t ISL_OVLO : 1;
		uint16_t ISL_UVLO : 1;
		uint16_t ISL_OV : 1;
		uint16_t ISL_UV : 1;

		uint16_t CellOverVoltage : 1;
		uint16_t CellUnderVoltage : 1;

		uint16_t rsvd:16-5-4-2;
	}bit;
};





typedef struct{
	float Vcellmin;
	float Vcellmax;
	float PackCurrent;
	float CellVoltages[8];
	float Vpack;
	float Temp1;
	float Temp2;

	BMS_gpio			gpio;
	BMS_settings		set;
	BMS_calculations 	cal;
	eState				state;

	union BMS_ALARMS	alarms;
	uint8_t				alarms_cnt[6];
	uint8_t				init;
	uint32_t			cnt[3];
	uint32_t			cntSleep;
	uint32_t			cntErrorSleep;
	uint8_t				cntStart;
	uint8_t				ButBlocked;
	uint8_t				AbsorbingCharge;
} BMS_LiIon;


typedef struct
{
  uint8_t sleep;
  uint8_t shutdown;
  char str_buf[10][50];

  char str_file[20];
  uint8_t cnt;
  float cnt_f;
  __IO uint8_t aShowTime[10];
  __IO uint8_t aShowDate[10];
  uint8_t timestamp[22];

  __IO uint8_t SDCardInit;
  __IO uint8_t SDCardSave;
  __IO uint8_t SDCardClose;
  __IO uint8_t SDCreatefile;
} CPU_TypeDef;
extern CPU_TypeDef cpu;

typedef enum
{
  BUTTON_USER = 0,
  /* Alias */
  BUTTON_KEY  = BUTTON_USER
} Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;


extern FATFS SDFatFs;  /* File system object for SD disk logical drive */
extern FIL MyFile;     /* File object */
extern char SDPath[4]; /* SD disk logical drive path */
extern uint8_t SDbuffer[_MAX_SS]; /* a work buffer for the f_mkfs() */
extern FRESULT res;                                          /* FatFs function common result code */
extern unsigned int byteswritten, bytesread;                     /* File write/read counts */

extern CPU_TypeDef 	cpu;
extern BMS_LiIon 	bms;



uint8_t BSP_SD_Init(void);
uint8_t BSP_SD_DeInit(void);
uint8_t BSP_SD_ITConfig(void);
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr);
uint8_t BSP_SD_GetCardState(void);
void    BSP_SD_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo);
uint8_t BSP_SD_IsDetected(void);

void    BSP_SD_MspInit(SD_HandleTypeDef *hsd1, void *Params);
void    BSP_SD_Detect_MspInit(SD_HandleTypeDef *hsd1, void *Params);
void    BSP_SD_MspDeInit(SD_HandleTypeDef *hsd1, void *Params);
void    BSP_SD_AbortCallback(void);
void    BSP_SD_WriteCpltCallback(void);
void    BSP_SD_ReadCpltCallback(void);


//RTC
void SystemPower_Config(void);
void RTC_CalendarConfig(void);
void RTC_CalendarShow(uint8_t *timestamp);
void RTC_TimeStampConfig(void);

//Low Power
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void Shutdown_detection();

void Init_SDcard();
void mainTask();
void timerTask();


#ifdef __cplusplus
}
#endif


#endif /* __MYDEVICE_H */
