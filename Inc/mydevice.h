
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYDEVICE_H
#define __MYDEVICE_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h"


extern  SD_HandleTypeDef hsd1;

#define BSP_SD_CardInfo HAL_SD_CardInfoTypeDef

#define MSD_OK                        ((uint8_t)0x00)
#define MSD_ERROR                     ((uint8_t)0x01)
#define MSD_ERROR_SD_NOT_PRESENT      ((uint8_t)0x02)


#define SD_TRANSFER_OK                ((uint8_t)0x00)
#define SD_TRANSFER_BUSY              ((uint8_t)0x01)


#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)

#define SD_DATATIMEOUT           ((uint32_t)100000000)


typedef struct
{
  uint8_t sleep;
  uint8_t shutdown;
  char str_buf1[50];
  char str_buf2[50];
  uint8_t cnt;
  float cnt_f;
  __IO uint8_t aShowTime[10];
  __IO uint8_t aShowDate[10];
  uint8_t timestamp[22];

  __IO uint8_t updateData;
  __IO uint8_t SDCardSave;
  __IO uint8_t SDCardClose;
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
