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
#include "cmsis_os.h"

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

#include "FreeRTOSConfig.h"

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

osThreadId Task01Handle;
osThreadId Task02Handle;
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

char buffer[20];
//************************Used in example 7****************************************
/* Declare a variable that will be incremented by the hook function. */
volatile uint32_t ulIdleCycleCount = 0UL;
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
//************************Used in example 1,3,4,5****************************************
//void StartTask01(void const * argument);
//Svoid StartTask02(void const * argument);
//************************Used in example 2, 7****************************************
//void vTaskFunction( void *pvParameters );
//************************Used in example 6****************************************
//void vContinuousProcessingTask( void *pvParameters );
//void vPeriodicTask( void *pvParameters );
//************************Used in example 8, 9****************************************
//void vTask1( void *pvParameters );
//void vTask2( void *pvParameters );
//************************Used in example 10****************************************
static void vSenderTask( void *pvParameters );
static void vReceiverTask( void *pvParameters );
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

/* Define the strings that will be passed in as the task parameters. These are
defined const and not on the stack to ensure they remain valid when the tasks are
executing. */

//************************Used in example 6****************************************
//static const char *pcTextForTask1 = "vContinuousProcessingTask 1 is running\r\n";
//static const char *pcTextForTask2 = "vContinuousProcessingTask 2 is running\r\n";

//************************Used in example 2,7****************************************
//static const char *pcTextForTask1 = "Task 1 is running\r\n";
//static const char *pcTextForTask2 = "Task 2 is running\r\n";

//************************Used in example 7, 8****************************************
/* Idle hook functions MUST be called vApplicationIdleHook(), take no parameters,
and return void. */
void vApplicationIdleHook( void )
{
/* This hook function does nothing but increment a counter. */
	ulIdleCycleCount++;
}

//************************Used in example 8****************************************
/* Declare a variable that is used to hold the handle of Task 2. */
//TaskHandle_t xTask2Handle = NULL;

//************************Used in example 10****************************************
/* Declare a variable of type QueueHandle_t. This is used to store the handle
to the queue that is accessed by all three tasks. */
QueueHandle_t xQueue;

//************************Used in example 11****************************************
/* Define an enumerated type used to identify the source of the data. */
typedef enum
{
	eSender1,
	eSender2
} DataSource_t;
/* Define the structure type that will be passed on the queue. */
typedef struct
{
	uint8_t ucValue;
	DataSource_t eDataSource;
} Data_t;
/* Declare two variables of type Data_t that will be passed on the queue. */
static const Data_t xStructsToSend[ 2 ] =
{
		{ 100, eSender1 }, /* Used by Sender1. */
		{ 200, eSender2 } /* Used by Sender2. */
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  MX_RTC_Init();
  MX_IWDG_Init();
  MX_WWDG_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
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

   //************************Used in example 1,3,4,5****************************************
  /* definition and creation of Task01 */
//  osThreadDef(Task01, StartTask01, osPriorityNormal, 0, 128);
//  Task01Handle = osThreadCreate(osThread(Task01), NULL);
//
////  /* definition and creation of Task02 */
//  osThreadDef(Task02, StartTask02, osPriorityHigh, 0, 128);
//  Task02Handle = osThreadCreate(osThread(Task02), NULL);

   //************************Used in example 2****************************************
  /* Create one of the two tasks. */
//  xTaskCreate( vTaskFunction, /* Pointer to the function that
//  	  	  	  	  	  	  	  implements the task. */
//		  	  	  "Task 1", /* Text name for the task. This is to
//  	  	  	  	  	  	  	  facilitate debugging only. */
//				  1000, /* Stack depth - small microcontrollers
//  	  	  	  	  	  	  will use much less stack than this. */
//				  (void*)pcTextForTask1, /* Pass the text to be printed into the
//  	  	  	  	  	  	  	  	  	  	  task using the task parameter. */
//				  1, /* This task will run at priority 1. */
//				  NULL ); /* The task handle is not used in this
//  example. */
//  /* Create the other task in exactly the same way. Note this time that multiple
//  tasks are being created from the SAME task implementation (vTaskFunction). Only
//  the value passed in the parameter is different. Two instances of the same
//  task are being created. */
//  xTaskCreate( vTaskFunction, "Task 2", 1000, (void*)pcTextForTask2, 2, NULL );

   //************************Used in example 6****************************************

//     xTaskCreate(vContinuousProcessingTask, "Task 1", 1000, (void*)pcTextForTask1, 1, NULL );
//     xTaskCreate(vContinuousProcessingTask, "Task 2", 1000, (void*)pcTextForTask2, 1, NULL );
//     xTaskCreate(vPeriodicTask, "Task 3", 1000, NULL, 2, NULL );

   //************************Used in example 8****************************************

   /* Create the first task at priority 2. The task parameter is not used
   and set to NULL. The task handle is also not used so is also set to NULL. */
   //xTaskCreate( vTask1, "Task 1", 1000, NULL, 2, NULL );
   /* The task is created at priority 2 ______^. */
   /* Create the second task at priority 1 - which is lower than the priority
   given to Task 1. Again the task parameter is not used so is set to NULL -
   BUT this time the task handle is required so the address of xTask2Handle
   is passed in the last parameter. */
   //xTaskCreate( vTask2, "Task 2", 1000, NULL, 1, &xTask2Handle );
   /* The task handle is the last parameter _____^^^^^^^^^^^^^ */

   //************************Used in example 9****************************************
   /* Create the first task at priority 1. The task parameter is not used
   so is set to NULL. The task handle is also not used so likewise is set
   to NULL. */
//   xTaskCreate( vTask1, "Task 1", 1000, NULL, 1, NULL );
   /* The task is created at priority 1 ______^. */

   //************************Used in example 10****************************************

//   /* The queue is created to hold a maximum of 5 values, each of which is
//   large enough to hold a variable of type int32_t. */
//   xQueue = xQueueCreate( 5, sizeof( int32_t ) );
//   if( xQueue != NULL )
//   {
//	   /* Create two instances of the task that will send to the queue. The task
//   	   parameter is used to pass the value that the task will write to the queue,
//   	   so one task will continuously write 100 to the queue while the other task
//   	   will continuously write 200 to the queue. Both tasks are created at
//   	   priority 1. */
//	   xTaskCreate( vSenderTask, "Sender1", 1000, ( void * ) 100, 1, NULL );
//	   xTaskCreate( vSenderTask, "Sender2", 1000, ( void * ) 200, 1, NULL );
//	   /* Create the task that will read from the queue. The task is created with
//   	   priority 2, so above the priority of the sender tasks. */
//	   xTaskCreate( vReceiverTask, "Receiver", 1000, NULL, 2, NULL );
//	   /* Start the scheduler so the created tasks start executing. */
//	   vTaskStartScheduler();
//   }
//   else
//   {
//	   /* The queue could not be created. */
//   }

   //************************Used in example 11****************************************
   /* The queue is created to hold a maximum of 3 structures of type Data_t. */
   xQueue = xQueueCreate( 3, sizeof( Data_t ) );
   if( xQueue != NULL )
   {
   /* Create two instances of the task that will write to the queue. The
   parameter is used to pass the structure that the task will write to the
   queue, so one task will continuously send xStructsToSend[ 0 ] to the queue
   while the other task will continuously send xStructsToSend[ 1 ]. Both
   tasks are created at priority 2, which is above the priority of the receiver. */
   xTaskCreate( vSenderTask, "Sender1", 1000, &( xStructsToSend[ 0 ] ), 2, NULL );
   xTaskCreate( vSenderTask, "Sender2", 1000, &( xStructsToSend[ 1 ] ), 2, NULL );
   /* Create the task that will read from the queue. The task is created with
   priority 1, so below the priority of the sender tasks. */
   xTaskCreate( vReceiverTask, "Receiver", 1000, NULL, 1, NULL );
   /* Start the scheduler so the created tasks start executing. */
   vTaskStartScheduler();
   }
   else
   {
   /* The queue could not be created. */
   }
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

//  /* Start scheduler */
//  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
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

//************************Used in example 1,3,4,5****************************************
/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
//void StartTask01(void const * argument)
//{
//  /* USER CODE BEGIN 5 */
//	TickType_t xLastWakeTime;
//
//	/* The xLastWakeTime variable needs to be initialized with the current tick
//	count. Note that this is the only time the variable is written to explicitly.
//	After this xLastWakeTime is automatically updated within vTaskDelayUntil(). */
//	xLastWakeTime = xTaskGetTickCount();
//  /* Infinite loop */
//  for(;;)
//  {
//	HAL_UART_Transmit(&huart2, (uint8_t *)"TASK-1 is running \n\r", sizeof("TASK-1 is running \n\r"), 1000);
//
//	osDelayUntil(&xLastWakeTime, 250);
//  }
//  /* USER CODE END 5 */
//}
//
///* USER CODE BEGIN Header_StartTask02 */
///**
//* @brief Function implementing the Task02 thread.
//* @param argument: Not used
//* @retval None
//*/
///* USER CODE END Header_StartTask02 */
//void StartTask02(void const * argument)
//{
//  /* USER CODE BEGIN StartTask02 */
//	TickType_t xLastWakeTime;
//
//	/* The xLastWakeTime variable needs to be initialized with the current tick
//	count. Note that this is the only time the variable is written to explicitly.
//	After this xLastWakeTime is automatically updated within vTaskDelayUntil(). */
//	xLastWakeTime = xTaskGetTickCount();
//  /* Infinite loop */
//  for(;;)
//  {
//	HAL_UART_Transmit(&huart2, (uint8_t *)"TASK-2 is running \n\r", sizeof("TASK-2 is running \n\r"), 1000);
//
//	osDelayUntil(&xLastWakeTime, 250);
//  }
//  /* USER CODE END StartTask02 */
//}

//************************Used in example 2, 7***************************************
//void vTaskFunction( void *pvParameters )
//{
//	char *pcTaskName;
//	const TickType_t xDelay250ms = pdMS_TO_TICKS( 250 );
//	/* The string to print out is passed in via the parameter. Cast this to a
//	character pointer. */
//	pcTaskName = ( char * ) pvParameters;
//
//    sprintf(buffer,"%s\n\r", pcTaskName);
//	/* As per most tasks, this task is implemented in an infinite loop. */
//	for( ;; )
//	{
//		/* Print out the name of this task. */
//		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), 1000);
//		/* Delay for a period. */
//		vTaskDelay(xDelay250ms);
//	}
//}

//************************Used in example 6****************************************
//void vContinuousProcessingTask( void *pvParameters )
//{
//	char *pcTaskName;
//	/* The string to print out is passed in via the parameter. Cast this to a
//	character pointer. */
//	pcTaskName = ( char * ) pvParameters;
//	sprintf(buffer,"%s", pcTaskName);
//	/* As per most tasks, this task is implemented in an infinite loop. */
//	for( ;; )
//	{
//			/* Print out the name of this task. This task just does this repeatedly
//			without ever blocking or delaying. */
//		HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), 1000);
//	}
//}
//
//void vPeriodicTask( void *pvParameters )
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xDelay3ms = pdMS_TO_TICKS( 3 );
//	/* The xLastWakeTime variable needs to be initialized with the current tick
//	count. Note that this is the only time the variable is explicitly written to.
//	After this xLastWakeTime is managed automatically by the vTaskDelayUntil()
//	API function. */
//	xLastWakeTime = xTaskGetTickCount();
//	/* As per most tasks, this task is implemented in an infinite loop. */
//	for( ;; )
//	{
//		/* Print out the name of this task. */
//		HAL_UART_Transmit(&huart2, (uint8_t *)"PeriodicTask is running\n\r", sizeof("PeriodicTask is running\n\r"), 1000);
//		/* The task should execute every 3 milliseconds exactly – see the
//		declaration of xDelay3ms in this function. */
//		vTaskDelayUntil( &xLastWakeTime, xDelay3ms );
//	}
//}

//************************Used in example 8****************************************
//void vTask1( void *pvParameters )
//{
//	UBaseType_t uxPriority;
//	/* This task will always run before Task 2 as it is created with the higher
//	priority. Neither Task 1 nor Task 2 ever block so both will always be in
//	either the Running or the Ready state.
//	Query the priority at which this task is running - passing in NULL means
//	"return the calling task’s priority". */
//	uxPriority = uxTaskPriorityGet( NULL );
//	for( ;; )
//	{
//		/* Print out the name of this task. */
//		HAL_UART_Transmit(&huart2, (uint8_t *)"Task 1 is running\r\n", sizeof("Task 1 is running\r\n"), 1000);
//		/* Setting the Task 2 priority above the Task 1 priority will cause
//	Task 2 to immediately start running (as then Task 2 will have the higher
//	priority of the two created tasks). Note the use of the handle to task
//	2 (xTask2Handle) in the call to vTaskPrioritySet(). Listing 35 shows how
//	the handle was obtained. */
//		HAL_UART_Transmit(&huart2, (uint8_t *)"About to raise the Task 2 priority\r\n", sizeof("About to raise the Task 2 priority\r\n"), 1000);
//		vTaskPrioritySet( xTask2Handle, ( uxPriority + 1 ) );
//		/* Task 1 will only run when it has a priority higher than Task 2.
//	Therefore, for this task to reach this point, Task 2 must already have
//	executed and set its priority back down to below the priority of this
//	task. */
//	}
//}
//
//void vTask2( void *pvParameters )
//{
//	UBaseType_t uxPriority;
//	/* Task 1 will always run before this task as Task 1 is created with the
//	higher priority. Neither Task 1 nor Task 2 ever block so will always be
//	in either the Running or the Ready state.
//	Query the priority at which this task is running - passing in NULL means
//	"return the calling task’s priority". */
//	uxPriority = uxTaskPriorityGet( NULL );
//	for( ;; )
//	{
//		/* For this task to reach this point Task 1 must have already run and
//	set the priority of this task higher than its own.
//	Print out the name of this task. */
//		HAL_UART_Transmit(&huart2, (uint8_t *) "Task 2 is running\r\n", sizeof( "Task 2 is running\r\n"), 1000);
//
//		/* Set the priority of this task back down to its original value.
//	Passing in NULL as the task handle means "change the priority of the
//	calling task". Setting the priority below that of Task 1 will cause
//	Task 1 to immediately start running again – pre-empting this task. */
//		HAL_UART_Transmit(&huart2, (uint8_t *)"About to lower the Task 2 priority\r\n", sizeof("About to lower the Task 2 priority\r\n"), 1000);
//
//		vTaskPrioritySet( NULL, ( uxPriority - 2 ) );
//	}
//}

//************************Used in example 9****************************************
//TaskHandle_t xTask2Handle = NULL;
//void vTask1( void *pvParameters )
//{
//	const TickType_t xDelay100ms = pdMS_TO_TICKS( 100UL );
//	for( ;; )
//	{
//		/* Print out the name of this task. */
//		HAL_UART_Transmit(&huart2, (uint8_t *)"Task 1 is running\r\n", sizeof("Task 1 is running\r\n"), 1000);
//		/* Create task 2 at a higher priority. Again the task parameter is not
//		used so is set to NULL - BUT this time the task handle is required so
//		the address of xTask2Handle is passed as the last parameter. */
//		xTaskCreate( vTask2, "Task 2", 1000, NULL, 2, &xTask2Handle );
//		/* The task handle is the last parameter _____^^^^^^^^^^^^^ */
//		/* Task 2 has/had the higher priority, so for Task 1 to reach here Task 2
//		must have already executed and deleted itself. Delay for 100
//		milliseconds. */
//		vTaskDelay( xDelay100ms );
//	}
//}
//
//void vTask2( void *pvParameters )
//{
//	/* Task 2 does nothing but delete itself. To do this it could call vTaskDelete()
//	using NULL as the parameter, but instead, and purely for demonstration purposes,
//	it calls vTaskDelete() passing its own task handle. */
//	HAL_UART_Transmit(&huart2, (uint8_t *)"Task 2 is running and about to delete itself\r\n", sizeof("Task 2 is running and about to delete itself\r\n"), 1000);
//	vTaskDelete( xTask2Handle );
//}

//************************Used in example 10****************************************
//static void vSenderTask( void *pvParameters )
//{
//	int32_t lValueToSend;
//	BaseType_t xStatus;
//	/* Two instances of this task are created so the value that is sent to the
//	queue is passed in via the task parameter - this way each instance can use
//	a different value. The queue was created to hold values of type int32_t,
//	so cast the parameter to the required type. */
//	lValueToSend = ( int32_t ) pvParameters;
//	/* As per most tasks, this task is implemented within an infinite loop. */
//	for( ;; )
//	{
//			/* Send the value to the queue.
//			The first parameter is the queue to which data is being sent. The
//			queue was created before the scheduler was started, so before this task
//			started to execute.
//			The second parameter is the address of the data to be sent, in this case
//			the address of lValueToSend.
//			The third parameter is the Block time – the time the task should be kept
//			in the Blocked state to wait for space to become available on the queue
//			should the queue already be full. In this case a block time is not
//			specified because the queue should never contain more than one item, and
//			therefore never be full. */
//			xStatus = xQueueSendToBack( xQueue, &lValueToSend, 0 );
//			if( xStatus != pdPASS )
//			{
//				/* The send operation could not complete because the queue was full -
//				this must be an error as the queue should never contain more than
//				one item! */
//				HAL_UART_Transmit(&huart2, (uint8_t *)"Could not send to the queue.\r\n", sizeof("Could not send to the queue.\r\n"), 1000);
//			}
//	}
//}
//
//static void vReceiverTask( void *pvParameters )
//{
//	/* Declare the variable that will hold the values received from the queue. */
//	int32_t lReceivedValue;
//	BaseType_t xStatus;
//	char buffer[20];
//	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
//	/* This task is also defined within an infinite loop. */
//	for( ;; )
//	{
//		/* This call should always find the queue empty because this task will
//		immediately remove any data that is written to the queue. */
//		if( uxQueueMessagesWaiting( xQueue ) != 0 )
//		{
//			HAL_UART_Transmit(&huart2, (uint8_t *)"Queue should have been empty!\r\n", sizeof("Queue should have been empty!\r\n"), 1000);
//		}
//		/* Receive data from the queue.
//		The first parameter is the queue from which data is to be received. The
//		queue is created before the scheduler is started, and therefore before this
//		task runs for the first time.
//		The second parameter is the buffer into which the received data will be
//		placed. In this case the buffer is simply the address of a variable that
//		has the required size to hold the received data.
//		The last parameter is the block time – the maximum amount of time that the
//		task will remain in the Blocked state to wait for data to be available
//		should the queue already be empty. */
//		xStatus = xQueueReceive( xQueue, &lReceivedValue, xTicksToWait );
//		if( xStatus == pdPASS )
//		{
//			/* Data was successfully received from the queue, print out the received
//			value. */
//			//vPrintStringAndNumber( "Received = ", lReceivedValue );
//			sprintf(buffer, "Recieved = %ld \n\r", lReceivedValue);
//			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, sizeof(buffer), 1000);
//		}
//		else
//		{
//			/* Data was not received from the queue even after waiting for 100ms.
//			This must be an error as the sending tasks are free running and will be
//			continuously writing to the queue. */
//			HAL_UART_Transmit(&huart2, (uint8_t *)"Could not receive from the queue.\r\n", sizeof("Could not receive from the queue.\r\n"), 1000);
//		}
//	}
//}

//************************Used in example 11****************************************
static void vSenderTask( void *pvParameters )
{
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );
	/* As per most tasks, this task is implemented within an infinite loop. */
	for( ;; )
	{
			/* Send to the queue.
			The second parameter is the address of the structure being sent. The
			address is passed in as the task parameter so pvParameters is used
			directly.
			The third parameter is the Block time - the time the task should be kept
			in the Blocked state to wait for space to become available on the queue
			if the queue is already full. A block time is specified because the
			sending tasks have a higher priority than the receiving task so the queue
			is expected to become full. The receiving task will remove items from
			the queue when both sending tasks are in the Blocked state. */
			xStatus = xQueueSendToBack( xQueue, pvParameters, xTicksToWait );
			if( xStatus != pdPASS )
			{
			/* The send operation could not complete, even after waiting for 100ms.
			This must be an error as the receiving task should make space in the
			queue as soon as both sending tasks are in the Blocked state. */
			HAL_UART_Transmit(&huart2, (uint8_t *)"Could not send to the queue.\r\n", sizeof("Could not send to the queue.\r\n"), 1000);
			}
	}
}

static void vReceiverTask( void *pvParameters )
{
	/* Declare the structure that will hold the values received from the queue. */
	char buffer1[20];
	char buffer2[20];
	Data_t xReceivedStructure;
	BaseType_t xStatus;
	/* This task is also defined within an infinite loop. */
	for( ;; )
	{
		/* Because it has the lowest priority this task will only run when the
		sending tasks are in the Blocked state. The sending tasks will only enter
		the Blocked state when the queue is full so this task always expects the
		number of items in the queue to be equal to the queue length, which is 3 in
		this case. */
		if( uxQueueMessagesWaiting( xQueue ) != 3 )
		{
			HAL_UART_Transmit(&huart2, (uint8_t *)"Queue should have been full!\r\n", sizeof("Queue should have been full!\r\n"), 1000);
		}
		/* Receive from the queue.
		The second parameter is the buffer into which the received data will be
		placed. In this case the buffer is simply the address of a variable that
		has the required size to hold the received structure.
		The last parameter is the block time - the maximum amount of time that the
		task will remain in the Blocked state to wait for data to be available
		if the queue is already empty. In this case a block time is not necessary
		because this task will only run when the queue is full. */
		xStatus = xQueueReceive( xQueue, &xReceivedStructure, 0 );
		if( xStatus == pdPASS )
		{
			/* Data was successfully received from the queue, print out the received
			value and the source of the value. */
			if( xReceivedStructure.eDataSource == eSender1 )
			{
				sprintf(buffer1, "From Sender 1 = %d \n\r", xReceivedStructure.ucValue);
				HAL_UART_Transmit(&huart2, (uint8_t *)buffer1, sizeof(buffer1), 1000);
				HAL_UART_Transmit(&huart2, (uint8_t *)"\n", sizeof("\n"), 1000);
			}
			else
			{
				sprintf(buffer2, "From Sender 2 = %d \n\r", xReceivedStructure.ucValue);
				HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, sizeof(buffer2), 1000);
				HAL_UART_Transmit(&huart2, (uint8_t *)"\n", sizeof("\n"), 1000);
			}
		}
		else
		{
		/* Nothing was received from the queue. This must be an error as this
		task should only run when the queue is full. */
		HAL_UART_Transmit(&huart2, (uint8_t *)"Could not receive from the queue.\r\n", sizeof("Could not receive from the queue.\r\n"), 1000);

		}
	}
}

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
