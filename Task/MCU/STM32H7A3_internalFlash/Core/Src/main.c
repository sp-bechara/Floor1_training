/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_hal_flash.h"
#include "string.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t GetSector(uint32_t Address)
{
	return (Address >> 13 ) & (0x7F);
}


void internalFlashReadData(uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{

		*RxBuf = *(__IO uint32_t *)StartSectorAddress;
		StartSectorAddress += 4;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
}

uint32_t internalFlashErase(uint32_t StartSectorAddress)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;

	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Erase the user Flash area */

	  /* Get the number of sector to erase from 1st sector */

	  uint32_t Sector = GetSector(StartSectorAddress);

	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
//	  EraseInitStruct.VoltageRange  = 3;
	  EraseInitStruct.Banks =  1;  //change this instruction if the bank is different
	  EraseInitStruct.Sector        = Sector;
	  EraseInitStruct.NbSectors     = 1;

	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	  	  {
	  		  return HAL_FLASH_GetError ();
	  	  }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();

	  return 0;
}

uint32_t internalFlashstartEndSectorEraseWrite(uint32_t BaseAddress, uint32_t *tempBuffer)
{
	int sofar=0;

	//Erase the start Sector
	internalFlashErase(BaseAddress);

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	//Start sector write
	while (sofar<2048+4)
	  	 {
	  	   if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, BaseAddress, tempBuffer) == HAL_OK)
	  	   {
	  	      BaseAddress += 16;
	  	      sofar += 4;
	  	      tempBuffer += 4;
	  	   }
	  	   else
	  	   {
	  	      /* Error occurred while writing data in Flash memory*/
	  	      return HAL_FLASH_GetError ();
	  	   }
	  	 }
	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  	 HAL_FLASH_Lock();

	return 0;
}

uint32_t internalFlashWriteData(uint32_t StartSectorAddress, uint32_t *Data,uint32_t dataBufferSize)
{

	 /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Get the sector detail */

	  uint32_t StartSector = GetSector(StartSectorAddress);
	  uint32_t EndSectorAddress = StartSectorAddress + dataBufferSize;
	  uint32_t EndSector = GetSector(EndSectorAddress);
	  uint32_t startSectorBaseAddress =  0x08000000/*It is base address of a bank 1*/ + (StartSector * 0x2000);
	  uint32_t startSectorEndAddress = startSectorBaseAddress + 0x1FFF/*It is size of sector*/;
	  uint32_t nextSectorBaseAddress=startSectorBaseAddress+0x2000; //It is a address of sector next to start sector
	  uint8_t noOfTimes = (EndSector-StartSector)+1; //It is used to count how much sector will be modified
	  uint8_t start=StartSector;//It is used to track the sector in which operation takes place
	  int sofar;
	  uint32_t tempBuffer[2048]={0x00};
	  while(noOfTimes > 0)
	  {
		  if(start==StartSector || start==EndSector)
		  	  {
		  		  if(start==StartSector)
		  		  {
		  			  //read the start sector data to temp buffer
		  			  internalFlashReadData(startSectorBaseAddress, tempBuffer, 2048);

		  			  //update a temp buffer
		  			  memcpy(&tempBuffer[(StartSectorAddress-startSectorBaseAddress)/4], Data, ((startSectorEndAddress-StartSectorAddress) > dataBufferSize) ? dataBufferSize : (startSectorEndAddress-StartSectorAddress));

		  			  //erase and write
		  			  internalFlashstartEndSectorEraseWrite(startSectorBaseAddress, tempBuffer);
		  		  }
		  		  else
		  		  {
		  			  uint32_t endSectorBaseAddress =  0x08000000/*It is base address of a bank 1*/ + (EndSector * 0x2000);

		  			  //read the end sector data to temp buffer
		  			  internalFlashReadData(endSectorBaseAddress, tempBuffer, 2048);

		  			  endSectorBaseAddress =  0x08000000/*It is base address of a bank 1*/ + (EndSector * 0x2000);

		  			  //update a temp buffer
		  			  memcpy(tempBuffer, Data + ((endSectorBaseAddress-StartSectorAddress)/4), (EndSectorAddress-endSectorBaseAddress));

		  			  //erase and write
		  			  internalFlashstartEndSectorEraseWrite(endSectorBaseAddress, tempBuffer);
		  		  }
		  		noOfTimes--;
		  		start++;
		  	  }
		  	  else
		  	  {
		  		sofar=0;
		  		//erase the sector
		  		internalFlashErase(nextSectorBaseAddress);

		  		Data += (startSectorEndAddress-StartSectorAddress)/4;

		  		/* Unlock the Flash to enable the flash control register access *************/
		  		HAL_FLASH_Unlock();

		  		//write a sector
		  		while (sofar<(((EndSector-StartSector)-1)*2048)+4)
		  			  	 {
		  			  	   if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, nextSectorBaseAddress, Data) == HAL_OK)
		  			  	   {
		  			  		  nextSectorBaseAddress += 16;
		  			  	      sofar += 4;
		  			  	      Data += 4;
		  			  	   }
		  			  	   else
		  			  	   {
		  			  	      /* Error occurred while writing data in Flash memory*/
		  			  	      return HAL_FLASH_GetError ();
		  			  	   }
		  			  	 }
				  /* Lock the Flash to disable the flash control register access (recommended
				     to protect the FLASH memory against possible unwanted operation) *********/
				  	 HAL_FLASH_Lock();

		  		noOfTimes -=((EndSector-StartSector)-1);
		  		start +=((EndSector-StartSector)-1);
		  		}
	  }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();

	   return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */


  const int arraySizeBytes = 8 * 1024;
  uint8_t myArray[8 * 1024];

  // Initialize the array with consecutive values
  for (int i = 0; i < arraySizeBytes; i++) {
      myArray[i] = (i % 256) + 1;
  }

  uint8_t Rx_Data[512];

  /*
   internalFlashWriteData() function takes 3 parameters:
   para1- Address to be passed of flash memory where you want to write/update the data
   para2- Buffer address to be passed which is to be written on a flash memory
   para3- size of buffer passed in para2
   */
  internalFlashWriteData(0x8020010 , (uint32_t *)myArray,8192);
  /*
     internalFlashReadData() function takes 3 parameters:
     para1- Address to be passed of flash memory where you want to read the data
     para2- Buffer address to be passed which is to be read data from a flash memory and stores it on a passed buffer
     para3- no of words to be read
     */
  internalFlashReadData(0x80FE000 , (uint32_t *)Rx_Data, 128);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_N_Pin USB_FS_P_Pin */
  GPIO_InitStruct.Pin = USB_FS_N_Pin|USB_FS_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
