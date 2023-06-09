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
#include "stm32f4xx_hal_rtc.h"
#include "stdio.h"
#ifdef I_2_C
#include "fonts.h"
#include "ssd1306.h"
#include "stm32f4xx_hal_i2c.h"
#endif //#ifdef I_2_C

#ifdef SPI
#include <string.h>
#endif //#ifdef SPI
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef R_T_C
#define MAX_TIME_STRING_LENGTH 35  // Maximum length for time string (including null terminator)
#endif //ifdef R_T_C

#ifdef ADC_DMA
#define VREFINT 1.21 //Internal reference voltage , V
#define ADCMAX 4095.0 //(2^12)-1 ADC max value
#define V25 0.76 //sensor voltage at 25 degree C
#define AVG_SLOPE 0.0025 //2.5mV/degree C
#define MAX_TEMPARETURE_LENGTH 22
#endif //#ifdef ADC_DMA

#ifdef ADC_POLL
#define ADCMAX 4095.0 //(2^12)-1 ADC max value
#define V25 0.76 //sensor voltage at 25 degree C
#define AVG_SLOPE 0.0025 //2.5mV/degree C
#define MAX_TEMPARETURE_LENGTH 22
#define VrefInt 3.3
#endif //#ifdef ADC_POLL

#ifdef ADC_IT
#define ADCMAX 4095.0 //(2^12)-1 ADC max value
#define V25 0.76 //sensor voltage at 25 degree C
#define AVG_SLOPE 0.0025 //2.5mV/degree C
#define MAX_TEMPARETURE_LENGTH 22
#define VrefInt 3.3
#endif //#ifdef ADC_IT

#ifdef SPI
#define BIT_NUMBER 7
#define POSITION_OF_BITS 9
#define POSITION_OF_BITS_FOR_BYTES_ADDRESS 0
#define NO_OF_BITS_TO_BE_EXTRACTED 8
#define NO_OF_BITS_TO_BE_EXTRACTED_FOR_BYTES_ADDRESS 8
#define PAGE_SIZE 256
#define BUFFER_SIZE 512
#define DEBUG_SPI_FLASH_CHIP_AT45DB081E 0
#define MEMORY_PAGE_READ_TX_SIZE 6
#define MEMORY_WRITE_ADDRESS 3583
#if (MEMORY_WRITE_ADDRESS < 0 || MEMORY_WRITE_ADDRESS > 4350)
    #error Invalid MEMORY_WRITE_ADDRESS value. It must be within the range of 0 to 4350.
#endif
#define MEMORY_READ_ADDRESS 3583
#if (MEMORY_READ_ADDRESS < 0 || MEMORY_READ_ADDRESS > 4350)
    #error Invalid MEMORY_READ_ADDRESS value. It must be within the range of 0 to 4350.
#endif
#endif //#ifdef SPI
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
#ifdef ECHOBACK
char RecievedData;
#endif //ifdef ECHOBACK

#ifdef R_T_C
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
RTC_AlarmTypeDef sAlarm;
char timeString[MAX_TIME_STRING_LENGTH]; // Time string
int RTC_Interrupt_flag=0;
#endif //ifdef R_T_C

#ifdef I_W_D_G
//IWDG_TIMEOUT value can be between 0 to 32768
#define IWDG_TIMEOUT 30000
#if (IWDG_TIMEOUT < 0 || IWDG_TIMEOUT > 32767)
    #error Invalid IWDG_TIMEOUT value. It must be within the range of 0 to 32767.
#endif
#define CLOCK_FREQUENCY 32000
#define RELOAD_RANGE 4096
int iwdgFlag=0;
uint32_t prescaler=0;
uint32_t prescalerIndex=0;
#endif //#ifdef I_W_D_G

#ifdef W_W_D_G
volatile int windowWatchdogInterruptFlag=0;
#endif //#ifdef W_W_D_G

#ifdef ADC_DMA
uint16_t adcRaw[2];
uint8_t adcConvCmplt = 0;
double temperature;
char tempBuffer[MAX_TEMPARETURE_LENGTH];
#endif //#ifdef ADC_DMA

#ifdef ADC_POLL
double temperature;
char tempBuffer[MAX_TEMPARETURE_LENGTH];
uint32_t adcVal;
#endif //#ifdef ADC_POLL

#ifdef ADC_IT
double temperature;
char tempBuffer[MAX_TEMPARETURE_LENGTH];
uint32_t adcVal;
int adcInterruptCheckFlag=0;
#endif //#ifdef ADC_IT

#ifdef SPI
#if DEBUG_SPI_FLASH_CHIP_AT45DB081E
uint8_t readRx[261];
#endif //#if DEBUG_SPI_FLASH_CHIP_AT45DB081E
uint8_t memoryPageReadRx[262];
uint8_t buffer[BUFFER_SIZE];
uint8_t userBuffer[BUFFER_SIZE];
#endif //#ifdef SPI
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);
static void MX_WWDG_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef R_T_C
void setInitialTime(void)
{

  /* Configure the RTC time structure */
  sTime.Hours = 00;
  sTime.Minutes = 00;
  sTime.Seconds = 00;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

  /* Configure the RTC date structure */
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 23;
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
}

void getCurrentTime(void)
{

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  /* Format the time and date */
  //used this sprintf for UART log
  //sprintf(timeString, "Time : %02d:%02d:%02d   Date : %02d/%02d/%02d", sTime.Hours, sTime.Minutes, sTime.Seconds, sDate.Date, sDate.Month, sDate.Year);

#ifdef I_2_C
  	//used this sprintf for I2C log
    sprintf(timeString,"%02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
#endif //#ifdef I_2_C
}

void setAlarm(void){
	sAlarm.Alarm = RTC_ALARM_A;
	sAlarm.AlarmTime.Hours = 00;          // Set the alarm hours
	sAlarm.AlarmTime.Minutes = 1;         // Set the alarm minutes
	sAlarm.AlarmTime.Seconds = 00;        // Set the alarm seconds
	sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;  // Set the alarm time format
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;  // Set the alarm to trigger on date
	sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;  // Set the alarm mask to trigger on date match
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;  // No sub-seconds comparison
	sAlarm.AlarmDateWeekDay = 1;           // Set the alarm date (1-31)
	sAlarm.Alarm = RTC_ALARM_A;            // Use Alarm A
	HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);  // Set and enable the alarm
}

void toDoOnAlarm(void)
{

	// Handle alarm event
	RTC_Interrupt_flag++;
}
#endif //ifdef R_T_C

#ifdef W_W_D_G
void watchdogFeed(void){
	HAL_WWDG_Refresh(&hwwdg);
	windowWatchdogInterruptFlag=0;
}
#endif //#ifdef W_W_D_G

#ifdef SPI
void spiChipSelect(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void spiChipDeselect(){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void isDeviceReadyAT45DB081E(){
	while(1){
		uint8_t rx_buffer[3] = {0, 0, 0};
		uint8_t tx_buffer[3] = {0xD7};
		spiChipSelect();
		HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 3, 1000);
		spiChipDeselect();
		if(((rx_buffer[1]>>BIT_NUMBER) & 1)==1){
#if DEBUG_SPI_FLASH_CHIP_AT45DB081E
			HAL_UART_Transmit(&huart2, (uint8_t *)"Device is ready\n", sizeof("Device is ready\n"), 1000);
#endif //#if DEBUG_SPI_FLASH_CHIP_AT45DB081E
			break;
		}
	}
}

void flashMemoryWriteAT45DB081E(uint32_t address, uint8_t* buffer, int noOfBytes)
{
    uint8_t commandTx[4] = {0x00};

    while (noOfBytes > 0) {

    	//Condition to check whether device is ready or not
    	isDeviceReadyAT45DB081E();

        // Transmit to the buffer
        commandTx[0]=0x84;
        commandTx[1]=0x00;
        commandTx[2]=0x00;
        commandTx[3]=0x00;
        spiChipSelect();
        HAL_SPI_Transmit(&hspi1, commandTx, sizeof(commandTx), HAL_MAX_DELAY);
        HAL_SPI_Transmit(&hspi1, buffer, (noOfBytes > PAGE_SIZE) ? PAGE_SIZE : noOfBytes, HAL_MAX_DELAY);
        spiChipDeselect();

        //Condition to check whether device is ready or not
        isDeviceReadyAT45DB081E();

#if DEBUG_SPI_FLASH_CHIP_AT45DB081E
        //Read data from buffer
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        uint8_t readTx[261] = {0xD4, 0x00, 0x00, 0x00, 0x00};
        HAL_SPI_TransmitReceive(&hspi1, readTx, readRx, 261, 1000);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
#endif //#if DEBUG_SPI_FLASH_CHIP_AT45DB081E

        //Buffer to Main Memory Page Program with Built-In Erase(at Page 0)
        commandTx[0] = 0x83;
        commandTx[1] = address >> 16;
        commandTx[2] = (address>>(POSITION_OF_BITS-1)) & ((1<<NO_OF_BITS_TO_BE_EXTRACTED)-1);
        spiChipSelect();
        HAL_SPI_Transmit(&hspi1, commandTx, sizeof(commandTx), 1000);
        spiChipDeselect();

        // Update the variables
        noOfBytes -= PAGE_SIZE;
        buffer += PAGE_SIZE;
        address += PAGE_SIZE;
    }
	//Condition to check whether device is ready or not
	isDeviceReadyAT45DB081E();
}

void flashMemoryReadAT45DB081E(uint32_t address, uint8_t* userBuffer, int readRange)
{
    int userBufferInput=0;
    int readRangeCheck=(readRange >= PAGE_SIZE) ? PAGE_SIZE+(MEMORY_PAGE_READ_TX_SIZE-1):readRange+7;
    uint8_t memoryPageReadTx[262] = {0x1B, address >> 16, (address >> (POSITION_OF_BITS - 1)) & ((1 << NO_OF_BITS_TO_BE_EXTRACTED) - 1), (address >> (POSITION_OF_BITS_FOR_BYTES_ADDRESS - 1)) & ((1 << NO_OF_BITS_TO_BE_EXTRACTED_FOR_BYTES_ADDRESS) - 1), 0x00, 0x00};

    while (readRangeCheck > 0) {
        spiChipSelect();
        memoryPageReadTx[1] = address >> 16;
        memoryPageReadTx[2] = (address>>(POSITION_OF_BITS-1)) & ((1<<NO_OF_BITS_TO_BE_EXTRACTED)-1);
        memoryPageReadTx[3] = (address >> (POSITION_OF_BITS_FOR_BYTES_ADDRESS - 1)) & ((1 << NO_OF_BITS_TO_BE_EXTRACTED_FOR_BYTES_ADDRESS) - 1);
        HAL_SPI_TransmitReceive(&hspi1, memoryPageReadTx, memoryPageReadRx, PAGE_SIZE+MEMORY_PAGE_READ_TX_SIZE, 1000);
        spiChipDeselect();
        readRangeCheck=(readRange >= PAGE_SIZE) ? PAGE_SIZE+(MEMORY_PAGE_READ_TX_SIZE-1)/*261*/ :readRange+7;
        for (int memoryPageReadRxInput = 6; memoryPageReadRxInput <= readRangeCheck; memoryPageReadRxInput++, userBufferInput++) {
        	userBuffer[userBufferInput] = memoryPageReadRx[memoryPageReadRxInput];
           }

        // Update the variables
        readRangeCheck -= PAGE_SIZE;
        readRange -= PAGE_SIZE;
        address += PAGE_SIZE;
        userBufferInput++;

        // Initialize buffer to zero
        bzero(memoryPageReadRx, sizeof(memoryPageReadRx));
    }
}
#endif //#ifdef SPI
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
  MX_USART2_UART_Init();
  //MX_RTC_Init();
  //MX_IWDG_Init();
  //MX_WWDG_Init();
  //MX_I2C1_Init();
  //MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
#ifdef I_W_D_G
  HAL_UART_Transmit(&huart2, (uint8_t *)"Watchdog is initialized\n", sizeof("Watchdog is initialized\n"), 1000);
#endif //#ifdef I_W_D_G

#ifdef ECHOBACK
  // Enable USART2 receive interrupt
  USART2->CR1 |= USART_CR1_RXNEIE;
#endif //ifdef ECHOBACK

#ifdef R_T_C
  /* Check if the RTC has been initialized */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
  {
      //set the time
      setInitialTime();
  }
  setAlarm();
#endif //ifdef R_T_C

#ifdef I_W_D_G
  // Get the current time
   uint32_t startTime = HAL_GetTick();
   uint32_t elapsedTime = 0;
#endif //#ifdef I_W_D_G

#ifdef I_2_C
   SSD1306_Init();
   SSD1306_Clear();

   //To print name on a display
//   SSD1306_GotoXY (0,0);
//   SSD1306_Puts ("Shrey", &Font_11x18, 1);
//   SSD1306_GotoXY (0, 30);
//   SSD1306_Puts ("Bechara", &Font_11x18, 1);
//   SSD1306_UpdateScreen();
#endif //#ifdef I_2_C

#ifdef ADC_DMA
   HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcRaw, 2);
   HAL_TIM_Base_Start(&htim3);
#endif //#ifdef ADC_DMA

#ifdef ADC_IT
   HAL_ADC_Start_IT(&hadc1);
#endif //#ifdef ADC_IT

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef R_T_C
	  if(RTC_Interrupt_flag!=0)
	  	      {
		  	  HAL_UART_Transmit(&huart2, (uint8_t *)"ALarm is called", sizeof("ALarm is called"), 1000);
	  	      RTC_Interrupt_flag=0;
	  	      }
	  /* Wait for a quick delay (e.g., 1 second) */
	  	  	  HAL_Delay(1000);
//
	  	      /* Get the current time from the RTC */
	 	  	  getCurrentTime();
//
//	      /* Print the timeS in Tera Term */
	      HAL_UART_Transmit(&huart2, (uint8_t *)timeString, MAX_TIME_STRING_LENGTH, 1000);
	      HAL_UART_Transmit(&huart2, (uint8_t *)"\n", sizeof("\n"), 1000);
#endif //ifdef R_T_C

#ifdef I_2_C
//	      SSD1306_Puts (timeString, &Font_11x18, 1);
//	      SSD1306_UpdateScreen();
//	      SSD1306_GotoXY (0,0);
#endif //#ifdef I_2_C

#ifdef I_W_D_G
	      // Get the elapsed time since starting
	          elapsedTime = HAL_GetTick() - startTime;

	      // Check if the first 30 seconds have elapsed
	          if (elapsedTime <= IWDG_TIMEOUT)
	          {
	            // Send signal to the watchdog
	        	HAL_UART_Transmit(&huart2, (uint8_t *)"Health is Okay..!\n", sizeof("Health is Okay..!\n"), 1000);
	        	HAL_Delay(1000);
	        	HAL_IWDG_Refresh(&hiwdg);
	          }
	          else{
	        	  HAL_UART_Transmit(&huart2, (uint8_t *)"Health signal is stopped..!\n", sizeof("Health signal is stopped..!\n"), 1000);
	        	  HAL_Delay(1000);
	      }
#endif //#ifdef I_W_D_G

#ifdef ADC_DMA
	          if(adcConvCmplt){
	        	  //Something do
	        	  double VrefInt = (VREFINT * ADCMAX)/adcRaw[0]; //it will give real supply voltage in microcontroller
	        	  double VTmpSens = (VrefInt*adcRaw[1])/ADCMAX; //it is used to check whether internal temp is running proper or not i.e. If its proper it's value will be similar to 0.76(sensor voltage at 25 degree C)
	        	  temperature = (VTmpSens - V25)/(AVG_SLOPE) + 25.0;
	        	  sprintf(tempBuffer,"%0.2lf", temperature);
	        	  HAL_UART_Transmit(&huart2, (uint8_t *)tempBuffer, strlen(tempBuffer), 1000);
	    	      HAL_UART_Transmit(&huart2, (uint8_t *)"\n", sizeof("\n"), 1000);
	    	      SSD1306_GotoXY (0,0);
	    	      SSD1306_Puts (tempBuffer, &Font_11x18, 1);
	    	      SSD1306_UpdateScreen();
	        	  adcConvCmplt=0;
	        	  HAL_Delay(1000);
	          }
#endif //#ifdef ADC_DMA

#ifdef ADC_POLL
	     HAL_ADC_Start(&hadc1);
	     HAL_ADC_PollForConversion(&hadc1, 100);
	     adcVal=HAL_ADC_GetValue(&hadc1);
	     HAL_ADC_Stop(&hadc1);
	     double VTmpSens = (VrefInt*adcVal)/ADCMAX; //it is used to check whether internal temp is running proper or not i.e. If its proper it's value will be similar to 0.76(sensor voltage at 25 degree C)
	     temperature = (VTmpSens - V25)/(AVG_SLOPE) + 25.0;
	     sprintf(tempBuffer,"%0.2lf", temperature);
	     HAL_UART_Transmit(&huart2, (uint8_t *)tempBuffer, strlen(tempBuffer), 1000);
	     HAL_UART_Transmit(&huart2, (uint8_t *)"\n", sizeof("\n"), 1000);
	     HAL_Delay(1000);
#endif //#ifdef ADC_POLL

#ifdef ADC_IT
	     if(adcInterruptCheckFlag){
	    	    sprintf(tempBuffer,"%0.2lf", temperature);
	    	    HAL_UART_Transmit(&huart2, (uint8_t *)tempBuffer, strlen(tempBuffer), 1000);
	    	    HAL_UART_Transmit(&huart2, (uint8_t *)"\n", sizeof("\n"), 1000);
	    	    adcInterruptCheckFlag=0;
	    	    HAL_Delay(1000);
	     }
#endif //#ifdef ADC_IT

#ifdef SPI
	 	//Condition to check whether device is ready or not
	     isDeviceReadyAT45DB081E();
	     break;
#endif //#ifdef SPI
  }
#ifdef SPI
  	 //To read the DeviceID, manufacturer details.
  	 uint8_t DeviceIDRxBuffer[6] = {0, 0, 0, 0, 0, 0};
  	 uint8_t DeviceIDTxBuffer[6] = {0x9F, 0, 0, 0, 0, 0};
  	 spiChipSelect();
	 HAL_SPI_TransmitReceive(&hspi1, DeviceIDTxBuffer, DeviceIDRxBuffer, 6, 1000);
	 spiChipDeselect();

	 //Filling the "buffer" and its size is BUFFER_SIZE
	 for (uint16_t bufferInput = 0; bufferInput < BUFFER_SIZE; bufferInput++) {
	         if (bufferInput <= 255) {
	             buffer[bufferInput] = (uint8_t)bufferInput;
	         } else {
	             buffer[bufferInput] = (uint8_t)(255 - (bufferInput - 256));
	         }
	     }
	 /* the parameter of memoryWrite is
	  para1 - Address of mainMemory, on which you want to write the buffer
	  para2 - Pass the "buffer" i.e. content you want to write to mainMemory
	  para3 - From para2 how much you want to write i.e. no of bytes
	  */

	 flashMemoryWriteAT45DB081E(MEMORY_WRITE_ADDRESS, buffer, 512);
	 /* the parameter of memoryRead and its size is SIZE
	  para1 - Address of mainMemory, from which you want to start to read
	  para2 - Pass the "userBuffer" i.e. content you want to read from mainMemory will be stored in this buffer
	  para3 - how much bytes you want to read
	  */
	 flashMemoryReadAT45DB081E(MEMORY_READ_ADDRESS, userBuffer, 2);
while(1){
}
#endif //#ifdef SPI
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */
#ifdef I_W_D_G
	prescaler = (CLOCK_FREQUENCY * (IWDG_TIMEOUT/1000)) / RELOAD_RANGE;
	uint8_t prescalerIndex;  // Default index
	// Perform bitwise right shift operations to determine the index
	prescalerIndex = (prescaler <= 4) ? 0 :
	                 (prescaler <= 8) ? 1 :
	                 (prescaler <= 16) ? 2 :
	                 (prescaler <= 32) ? 3 :
	                 (prescaler <= 64) ? 4 :
	                 (prescaler <= 128) ? 5 : 6;
	uint32_t reloadValue=((IWDG_TIMEOUT * CLOCK_FREQUENCY) / (4 * (1 << prescalerIndex) * 1000)) - 1;
#endif //#ifdef I_W_D_G
  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 3749;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
#ifdef ECHOBACK
	huart2.RxXferSize = 512;
#endif //#ifdef ECHOBACK
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef ADC_DMA
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//Below condition is used to know that from which ADC, interrupt is come
	if(hadc->Instance == ADC1){
		adcConvCmplt=255;
	}
}
#endif //#ifdef ADC_DMA

#ifdef ADC_IT
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    adcVal=HAL_ADC_GetValue(&hadc1);
    double VTmpSens = (VrefInt*adcVal)/ADCMAX; //it is used to check whether internal temp is running proper or not i.e. If its proper it's value will be similar to 0.76(sensor voltage at 25 degree C)
    temperature = (VTmpSens - V25)/(AVG_SLOPE) + 25.0;
    adcInterruptCheckFlag++;
}
#endif //#ifdef ADC_IT

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
