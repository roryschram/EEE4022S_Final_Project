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
#include <stdio.h>
#include "minmea.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "bmp280.h"
#include <float.h>
#include <math.h>
#include <LoRa.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INDENT_SPACES "  "

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t gps_raw[512] = {"\0"};
uint8_t flag = 0;
char line_out[512] = {"\0"};
char line[512] = {"\0"};
bool bme280p;
uint8_t transmit_data[256] = {"\0"};
LoRa myLoRa;
bool isLoraReady = true;
int preTicks = 0;
int currentTicks = 0;

//Variables to work out altitude

//Pressure at sea level
float P_b = 101325;

//Height that we are calculating
float altitude = 0;


BMP280_HandleTypedef bmp280;

float pressure, temperature, humidity;
char sPressure[128] = {"\0"};
char sTemperature[128] = {"\0"};
char sHumidity[128] = {"\0"};
char sAltitude[128] = {"\0"};

uint16_t size;
uint8_t Data[256] = {"\0"};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  bmp280_init_default_params(&bmp280.params);
	bmp280.addr = 0x77;
	bmp280.i2c = &hi2c1;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		size = sprintf((char *)Data, "BMP280 initialization failed\n");
		HAL_UART_Transmit(&huart2, Data, size, 1000);
		HAL_Delay(2000);
	}
	bool bme280p = bmp280.id == BME280_CHIP_ID;
	size = sprintf((char *)Data, "\nBMP280: found %s\n", bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&huart2, Data, size, 1000);

  myLoRa = newLoRa();
  myLoRa.CS_port         = GPIOA;
  myLoRa.CS_pin          = GPIO_PIN_4;
  myLoRa.reset_port      = GPIOB;
  myLoRa.reset_pin       = GPIO_PIN_8;
  myLoRa.DIO0_port       = GPIOB;
  myLoRa.DIO0_pin        = GPIO_PIN_9;
  myLoRa.hSPIx           = &hspi3;

  uint16_t ret = LoRa_init(&myLoRa);
  uint8_t buff[128] = {"\0"};



if (ret==LORA_OK){
  snprintf(buff,sizeof(buff),"LoRa is running... :) \n\r");
  LoRa_transmit(&myLoRa, (uint8_t*)buff, 120, 100);
  HAL_UART_Transmit(&huart2, buff, sizeof(buff)/sizeof(buff[0]), 1000);
}
else{
  snprintf(buff,sizeof(buff),"\n\r LoRa failed :( \n\r Error code: %d \n\r", ret);
  HAL_UART_Transmit(&huart2, buff, sizeof(buff)/sizeof(buff[0]), 1000);
}

myLoRa.frequency             = 433;             // default = 433 MHz
myLoRa.spredingFactor        = SF_7;            // default = SF_7
myLoRa.bandWidth             = BW_7_8KHz;       // default = BW_125KHz
myLoRa.crcRate               = CR_4_8;          // default = CR_4_5
myLoRa.power                 = POWER_20db;      // default = 20db
myLoRa.overCurrentProtection = 100;             // default = 100 mA
myLoRa.preamble              = 8;              // default = 8;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t*)gps_raw,512);
    if (isLoraReady){
      HAL_UARTEx_ReceiveToIdle_IT(&huart1,(uint8_t*)gps_raw,512);
      if (flag == 1) {
      LoRa_transmit(&myLoRa, (uint8_t*)transmit_data, 113, 100);
      flag = 0;
      memset(transmit_data, '\0', sizeof(transmit_data));
    }
    }
    
    
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //HAL_UART_Transmit_IT(&huart2, (uint8_t *)Gpsdata, sizeof(Gpsdata));
    //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    //HAL_Delay(1000); 

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : rightButton_Pin */
  GPIO_InitStruct.Pin = rightButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(rightButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : leftButton_Pin */
  GPIO_InitStruct.Pin = leftButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(leftButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  //Set a flag high when the GPS sends data
  flag = 1;

  //Set memory of line_out to emtpy
  //memset(line_out, '\0', sizeof(line_out));
  memset(transmit_data, '\0', sizeof(transmit_data));

  //Copy a new line character to line_out
  //memcpy(line_out, "\r\n\n", sizeof("\r\n\n"));
  //memcpy(transmit_data, "\r\n\n", sizeof("\r\n\n"));

  //Transmit the data over uart2
  //HAL_UART_Transmit(&huart2, (uint8_t*)line_out, sizeof(line_out)/sizeof(line_out[0]), 1000);


  
  //Search for the \n character
  uint8_t *token = strtok(gps_raw, "\n");

  if (token == NULL) {
    //break
  } else {
    //memset(line, '\0', sizeof(line));
    strcpy(line, token);
    struct minmea_sentence_rmc frame;
    if (minmea_parse_rmc(&frame, line)) {
      //memset(line_out, '\0', sizeof(line_out));
      sprintf(transmit_data + strlen(transmit_data),"%d:%d:%d,",frame.time.hours, frame.time.minutes,frame.time.seconds);
      //HAL_UART_Transmit(&huart2, (uint8_t*)line_out, sizeof(line_out)/sizeof(line_out[0]), 1000);

      //memset(line_out, '\0', sizeof(line_out));
      sprintf(transmit_data + strlen(transmit_data), "(%d,%d),",minmea_rescale(&frame.latitude, 1000),minmea_rescale(&frame.longitude, 1000));
      //HAL_UART_Transmit(&huart2, (uint8_t*)line_out, sizeof(line_out)/sizeof(line_out[0]), 1000);
      
    }
  }

  //Set gps_raw data register to empty
  memset(gps_raw, '\0', sizeof(gps_raw));



  	//HAL_Delay(100);
	while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
	  size = sprintf(transmit_data + strlen(transmit_data),"Temperature/pressure reading failed\n");
	  //HAL_UART_Transmit(&huart2, Data, size, 1000);
	}




  //altitude = H_b + (T_b/L_b)*(pow((pressure/P_b),(-1*R*L_b)/(G_0*M))-1);

  altitude = 44330*(1-pow((pressure/P_b),(1/5.255)));

  gcvt(altitude, 6, sAltitude);
  gcvt(pressure, 8, sPressure);
  gcvt(temperature, 4, sTemperature);
  gcvt(humidity, 4, sHumidity);

	size = sprintf(transmit_data + strlen(transmit_data),"%s,%s,%s,%s",sPressure, sTemperature, sHumidity, sAltitude);
	HAL_UART_Transmit(&huart2, transmit_data, sizeof(transmit_data)/sizeof(transmit_data[0]), 1000);



  //Set flag low to indicate that data transfer is done
  
  //Send interrupt when data from GPS is sent again
  //HAL_UARTEx_ReceiveToIdle_IT(&huart1,(uint8_t*)gps_raw, 512);

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13) // INT Source is pin A1
    {
      //get the current time
      currentTicks = HAL_GetTick();
      if ((currentTicks-preTicks)>250) {
        memset(transmit_data, '\0', sizeof(transmit_data));
        sprintf(transmit_data,"%s","\nLeft Button Pushed");
	      HAL_UART_Transmit(&huart2, transmit_data, sizeof(transmit_data)/sizeof(transmit_data[0]),1000);
        memset(transmit_data, '\0', sizeof(transmit_data));
      } else {
        //break
      }
    }

  if(GPIO_Pin == GPIO_PIN_5) // INT Source is pin A1
    {
      //get the current time
      currentTicks = HAL_GetTick();
      if ((currentTicks-preTicks)>250) {
        memset(transmit_data, '\0', sizeof(transmit_data));
        sprintf(transmit_data,"%s","\nRight Button Pushed");
	      HAL_UART_Transmit(&huart2, transmit_data, sizeof(transmit_data)/sizeof(transmit_data[0]),1000);
        memset(transmit_data, '\0', sizeof(transmit_data));
      } else {
        //break
      }
    }  
  preTicks = currentTicks;
}

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
