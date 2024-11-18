
/* Includes ------------------------------------------------------------------*/

#include "mydevice.h"
#include "main.h"
#include "rng.h"
#include "sdmmc.h"
#include <string>
#include <bits/stdc++.h>
#include <iostream>
#include <ff.h>

using namespace std;

CPU_TypeDef cpu;







uint8_t BSP_SD_Init(void)
{
//	MX_SDMMC1_SD_Init();
  return  MSD_OK;
}

#define SD_DETECT_PIN                        ((uint32_t)GPIO_PIN_5)
#define SD_DETECT_GPIO_PORT                  ((GPIO_TypeDef*)GPIOB)

uint8_t BSP_SD_IsDetected(void)
{
  __IO uint8_t  status = SD_PRESENT;

  /* Check SD card detect pin */
//  if (HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) == GPIO_PIN_SET)
//  {
//    status = SD_NOT_PRESENT;
//  }
//  status = SD_PRESENT;
  return status;
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @param  Timeout: Timeout for read operation
  * @retval SD status
  */
uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  if(HAL_SD_ReadBlocks(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout) != HAL_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in polling mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @param  Timeout: Timeout for write operation
  * @retval SD status
  */
uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  if(HAL_SD_WriteBlocks(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout) != HAL_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/**
  * @brief  Reads block(s) from a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  ReadAddr: Address from where data is to be read
  * @param  NumOfBlocks: Number of SD blocks to read
  * @retval SD status
  */
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  /* Read block(s) in DMA transfer mode */
  if(HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/**
  * @brief  Writes block(s) to a specified address in an SD card, in DMA mode.
  * @param  pData: Pointer to the buffer that will contain the data to transmit
  * @param  WriteAddr: Address from where data is to be written
  * @param  NumOfBlocks: Number of SD blocks to write
  * @retval SD status
  */
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  /* Write block(s) in DMA transfer mode */
  if(HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/**
  * @brief  Erases the specified memory area of the given SD card.
  * @param  StartAddr: Start byte address
  * @param  EndAddr: End byte address
  * @retval SD status
  */
uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
  if(HAL_SD_Erase(&hsd1, StartAddr, EndAddr) != HAL_OK)
  {
    return MSD_ERROR;
  }
  else
  {
    return MSD_OK;
  }
}

/**
  * @brief  Initializes the SD MSP.
  * @param  hsd1: SD handle
  * @param  Params : pointer on additional configuration parameters, can be NULL.
  */
__weak void BSP_SD_MspInit(SD_HandleTypeDef *hsd1, void *Params)
{

}

/**
  * @brief  Initializes the SD Detect pin MSP.
  * @param  hsd1: SD handle
  * @param  Params : pointer on additional configuration parameters, can be NULL.
  */
__weak void BSP_SD_Detect_MspInit(SD_HandleTypeDef *hsd1, void *Params)
{

}

/**
  * @brief  DeInitializes the SD MSP.
  * @param  hsd1: SD handle
  * @param  Params : pointer on additional configuration parameters, can be NULL.
  */
__weak void BSP_SD_MspDeInit(SD_HandleTypeDef *hsd1, void *Params)
{

}

/**
  * @brief  Gets the current SD card data status.
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t BSP_SD_GetCardState(void)
{
  return((HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER ) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}


/**
  * @brief  Get SD information about specific SD card.
  * @param  CardInfo: Pointer to HAL_SD_CardInfoTypedef structure
  * @retval None
  */
void BSP_SD_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo)
{
  /* Get SD card Information */
  HAL_SD_GetCardInfo(&hsd1, CardInfo);
}

/**
  * @brief SD Abort callbacks
  * @param hsd1: SD handle
  * @retval None
  */
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd1)
{
  BSP_SD_AbortCallback();
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd1: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd1)
{
  BSP_SD_WriteCpltCallback();
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd1: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd1)
{
  BSP_SD_ReadCpltCallback();
}

/**
  * @brief BSP SD Abort callbacks
  * @retval None
  */
__weak void BSP_SD_AbortCallback(void)
{

}

/**
  * @brief BSP Tx Transfer completed callbacks
  * @retval None
  */
__weak void BSP_SD_WriteCpltCallback(void)
{

}

/**
  * @brief BSP Rx Transfer completed callbacks
  * @retval None
  */
__weak void BSP_SD_ReadCpltCallback(void)
{

}





/**
  * @brief  System Power Configuration
  * @retval None
  */
void SystemPower_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

      /* Set all GPIO in analog state to reduce power consumption */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = GPIO_PIN_All;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();

  /* Enable PWR clock */
  __HAL_RCC_PWR_CLK_ENABLE();
}


string tekst, tekst2, fulltext, sTimestamp;


void Shutdown_detection(){
	  /* Enable Power Clock */
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();

	/* Check if the system was resumed from shutdown mode,
	   resort to RTC back-up register RTC_BKP31R to verify
	   whether or not shutdown entry flag was set by software
	   before entering shutdown mode.  */
	if (READ_REG(RTC->BKP31R) == 1)
	{
	   WRITE_REG( RTC->BKP31R, 0x0 );  /* reset back-up register */
	   printf("Shutdown mode detection...\r\n");
	}

	/* Check and Clear the Wakeup flag */
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2) != RESET)
	{
	 __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
	}

}


FATFS SDFatFs;  /* File system object for SD disk logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD disk logical drive path */
uint8_t SDbuffer[_MAX_SS]; /* a work buffer for the f_mkfs() */
FRESULT res;                                          /* FatFs function common result code */
unsigned int byteswritten, bytesread;                     /* File write/read counts */



void Init_SDcard(){

	/*##-1- Link the SD disk I/O driver ########################################*/
	if(FATFS_LinkDriver(&SD_Driver, SDPath) != FR_OK)	return;
	else
	{
	printf("Init SD card...\r\n");
	// Register the file system object to the FatFs module
	 if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK) return;
	 else
	 {
	   //Create a FAT file system (format) on the logical drive
	   //if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, SDbuffer, sizeof(SDbuffer)) != FR_OK) Error_Handler();
	   //else
	   //{
		 /*##-4- Create and Open a new text file object with write access #####*/
		 if(f_open(&MyFile, "STM32L4.TXT", FA_CREATE_NEW | FA_WRITE) != FR_OK) printf("Error create STM32L4.TXT\r\n");
		 else printf("Done init SD card\r\n");

	 //  }
	 }
	}
	//timerTask();
}


void mainTask(){
//	 osDelay(500);
/*
	    if (cpu.sleep){
	    	cpu.sleep = 0;
	        // Configure the system Power
	        SystemPower_Config();

	        BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	        //Suspend Tick increment to prevent wakeup by Systick interrupt.
	        //Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)
	    	HAL_SuspendTick();
	    	//Enter Sleep Mode , wake up is done once User push-button is pressed
	    	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	    	//Resume Tick interrupt if disabled prior to SLEEP mode entry
	    	HAL_ResumeTick();
	    	// Re-configure GPIO
	    	MX_GPIO_Init();
	    	MX_DMA_Init();
	    	MX_USART2_UART_Init();
	    }
	    if (cpu.updateData){
	    	RTC_CalendarConfig();
	    }


	    if (cpu.SDCardClose){
	    	cpu.SDCardSave = 0;
	    	cpu.SDCardClose = 0;
	    	f_close(&MyFile);
	    	FATFS_UnLinkDriver(SDPath);
	    }


	    if (cpu.shutdown){
	    	cpu.shutdown = 0;
	        // Disable all used wakeup sources: WKUP pin
	        HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
	        // Clear wake up Flag
	        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
	        // Enable wakeup pin WKUP2
	        HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_LOW);
	        WRITE_REG( RTC->BKP31R, 0x1 );
	        HAL_PWREx_EnterSHUTDOWNMode();
	    }
*/
}


void timerTask(){

//	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
//	RTC_CalendarShow(cpu.timestamp);
	cpu.cnt_f+=0.1;
	cpu.cnt+=1;

	fulltext = "Ala ma kota" + to_string(cpu.cnt) + "\r";


//	stringstream myStreamString;
//	myStreamString << cpu.timestamp;
//	sTimestamp = myStreamString.str();

	//strcpy(cpu.str_buf1, sTimestamp.c_str());
	strcpy(cpu.str_buf2, fulltext.c_str());


	printf("Welcome from BMS_WIBAR\r\n");
	printf((char *)cpu.str_buf2);


	if (cpu.SDCardSave){
	//	f_write(&MyFile, cpu.str_buf1, sTimestamp.length(), &byteswritten);
		res = f_write(&MyFile, cpu.str_buf2, fulltext.length(), &byteswritten);
		cpu.SDCardSave = 0;
	}

	if (cpu.SDCardClose){
		cpu.SDCardSave = 0;
		cpu.SDCardClose = 0;
		f_close(&MyFile);
		//FATFS_UnLinkDriver(SDPath);
		printf("Close SD card\r\n");
	}
	if (cpu.SDCreatefile){
		string file_name = to_string(hrng.Instance->DR) + ".txt";
		strcpy(cpu.str_file, file_name.c_str());
		cpu.SDCreatefile = 0;
		if(f_open(&MyFile, cpu.str_file, FA_CREATE_NEW | FA_WRITE) != FR_OK) printf("Create SD file SD\r\n");
		else printf("Error create file SD\r\n");
	}


	if (cpu.SDCardInit){
		cpu.SDCardInit = 0;
		Init_SDcard();
	}


}


