/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MPU6050_ADDR 0x68<<1
#define PWR_MGMT_1_REG 0x6B
#define GYRO_CNFG_REG 0x1B
#define ACC_CNFG_REG 0x1C
#define LPF_REG 0x1A


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/**MPU6050 VERILERI**/
uint8_t data;
uint8_t temp_buffer[2],acc_buffer[6],gyro_buffer[6]; // ham gyro ve acc değerlerinin tutulduğu listeler
int16_t gyro_raw[3],acc_raw[3];        // anlamlandırılmış ham değerler
float gyro_cal[3],acc_cal[3]; // kalibrasyon offsetlerini tutan liste
int16_t acc_total_vector;
float angle_pitch_gyro, angle_roll_gyro, angle_yaw_gyro;
float angle_pitch_acc, angle_roll_acc,angle_yaw_acc, angle_pitch_acc_sum, angle_roll_acc_sum,acc_total_vector_sum;
float angle_pitch,angle_roll;
int16_t raw_temp;
int temp;
int i;
float prevtime,prevtime1,time1,elapsedtime1,prevtime2,time2,elapsedtime2;
HAL_StatusTypeDef set_gyro;

/**RTC VERILERI**/
char line1[32], line2[32];
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;
  uint16_t gun,ay,yil;
  uint16_t saat,dakika,saniye;

  /**ESP8266 VERILERI**/
  char txdata[150];
  char rxdata[150];
  char txdata1[31];
  char txdata2[29];
  char txdata3[32];
  char txdata4[35];




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

       /*****************************************MPU6050 GYRO+ACC SENSOR***********************************************************/
  // PWR_MGMT_1 CNFG ----> GUC AYARLAMASI YAPILIR
      data = 0x00;
      HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
      //kullanılan birimin pointer olarak döndürülmesi, cihaz slave adresi, register adresi, register boyutu, verinin pointer
      //olarak döndürülmesi, verinin boyutu , zaman aşımıı değeri

      // GYRO CNFG --> +-500 derece/saniye --> 08 (AÇISAL HIZ AYARLAMASI YAPILIR)
      data = 0x08;
      HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, GYRO_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);

      // ACC CNFG --> +-8g --> 10 (IVMENIN HIZLANMA AYARLAMASI YAPILIR)
      data = 0x10;
      HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, ACC_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);

     // DLPF_CFG---> ALCAK GECIREN FILTRE AYARLAMASI YAPILIR
      data = 0x03;
      HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, LPF_REG, 1, &data, 1, HAL_MAX_DELAY);

/* Offset islemi yapılarak daha temiz veriler alinir. Ortalama Metodu ile veriler for dongusune sokulur,
  degerler toplanır ardından ornek sayısına bolunerek ortalaması alınır.*/
      for(i=0; i<2000; i++)
      {

// prevtime ile her while dongusune girdigimizde islemde gecen sureyi gormemizi saglar//
    	  prevtime2 = time2;
    	  time2 = HAL_GetTick();
    	  elapsedtime2=(time2-prevtime2)*1000;

    	  gyro_buffer[0]=0x43; //GYRO OUTPUT REGISTER BASLANGIC ADRESI[0x43-0x48]//
    	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,gyro_buffer,1,HAL_MAX_DELAY);
    	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,gyro_buffer,6,HAL_MAX_DELAY);


// 8 bitlik sola kaydırma islemi yapılarak 16 bitlik degiskenle alanı kaplaması engellenir.//
// OR mantıksal operatoru ile de sonra gelecek verinin sagdaki bitte tutulmasını saglayıp  soldaki verinin gitmesini engellemis olur//
// 0x43 baslangıc registeri boylelikle [15:8] indexleri arasında degil [7:0] indexleri arası verileri yazmaya baslamıs olur//
    	  gyro_raw[0] = (gyro_buffer[0] << 8 | gyro_buffer[1]);
    	  gyro_raw[1] = (gyro_buffer[2] << 8 | gyro_buffer[3]);
    	  gyro_raw[2] = (gyro_buffer[4] << 8 | gyro_buffer[5]);


    	  // gyro_raw ile toplanmıs verileri gyro_cal dizisine gonderirilir.//
    	  gyro_cal[0] += gyro_raw[0];
    	  gyro_cal[1] += gyro_raw[1];
    	  gyro_cal[2] += gyro_raw[2];

    	  HAL_Delay(3); // olcum aralıgı 4ms olarak belirlenip ustteki kod 1ms'de calıstıgı icin 3ms gecikme eklenir//

      }
// toplanmıs verilerin ortalama degeri alınır//
      gyro_cal[0] /= 2000;
      gyro_cal[1] /= 2000;
      gyro_cal[2] /= 2000;

      HAL_Delay(1000);
      /*****************************************MPU6050 GYRO+ACC SENSOR***********************************************************


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*****************************************MPU6050 GYRO+ACC SENSOR***********************************************************/

	  // ustte gecmesı gereken 4ms'nin gecip gecmedigini anlamak icin kullanılır//
	  prevtime1 = time1;
	  time1 = HAL_GetTick();
	  elapsedtime1=(time1-prevtime1)*1000;

	 	 	  acc_buffer[0]=0x3B; //ACC OUTPUT REGISTER BASLANGIC ADRESI[0x3B-0x40]
	 	 	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,acc_buffer,1,HAL_MAX_DELAY);
	 	 	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,acc_buffer,6,HAL_MAX_DELAY);

	 	 	  // Acc Raw(ham) degerler)
	 	 	  acc_raw[0] = (acc_buffer[0] << 8 | acc_buffer[1]);
	 	 	  acc_raw[1] = (acc_buffer[2] << 8 | acc_buffer[3]);
	 	 	  acc_raw[2] = (acc_buffer[4] << 8 | acc_buffer[5]);


	 	 	  temp_buffer[0]=0x41;// TEMP OUTPUT REGISTER BASLANGIC ADRESI[0x41-0x42]//
	 	 	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,temp_buffer,1,HAL_MAX_DELAY);
	 	 	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,temp_buffer,2,HAL_MAX_DELAY);

	 	 	  // Temperature Values
	 	 	  raw_temp = (temp_buffer[0] << 8 | temp_buffer[1]);
	 	 	  temp = (raw_temp / 340.0) + 32.53;
	 	 	  //formul +36.53 gostermekte fakat 32.53'te daha iyi degerler elde edildi//


	 	 	  gyro_buffer[0]=0x43;
	 	 	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,gyro_buffer,1,HAL_MAX_DELAY);
	 	 	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,gyro_buffer,6,HAL_MAX_DELAY);

	 	 	  // Gyro Raw Values GYRO OUTPUT
	 	 	  gyro_raw[0] = (gyro_buffer[0] << 8 | gyro_buffer[1]);
	 	 	  gyro_raw[1] = (gyro_buffer[2] << 8 | gyro_buffer[3]);
	 	 	  gyro_raw[2] = (gyro_buffer[4] << 8 | gyro_buffer[5]);

	 	 	  //Olculen veri kalibre veriden çıkarılacak raw(ham) veriden daha duzgun deger alınması saglanır//
	 	 	  gyro_raw[0] -= gyro_cal[0];
	 	 	  gyro_raw[1] -= gyro_cal[1];
	 	 	  gyro_raw[2] -= gyro_cal[2];

              // gyro integral islemine tabi tutulur, elde edilen degerle carpılır.
	 	 	  //0.00006154 = (1/65.5)*0.004
	 	 	  angle_roll_gyro += gyro_raw[0] * 0.00006154; // roll---> X Ekseni
	 	 	  angle_pitch_gyro += gyro_raw[1] * 0.00006154; // pitch--> Y ekseni
	 	 	  angle_yaw_gyro += gyro_raw[2] * 0.00006154; // yaw--> z ekseni


	 	 	  //Radyan Dereceye cevrilir Derece= 180/3.14 = 57.296 ---> 0.000001066 = (0.00006154/57.296)
	 	 	  // z ekseni ham verisi ile x ve y ekseni hesaplamaları yapılır//
	 //eksenler birbiri üzerinde arttırıcı/azaltıcı etkende oldukları için birbirlerinden toplanır/cıkarılır
	 	 	  angle_pitch_gyro += angle_roll_gyro * sin(gyro_raw[2] * 0.000001066);
	 	 	  angle_roll_gyro -= angle_pitch_gyro * sin(gyro_raw[2] * 0.000001066);


              // acctotal=√x^2+y^2+z^2
	 	 	  acc_total_vector = sqrt((acc_raw[0]*acc_raw[0])+(acc_raw[1]*acc_raw[1])+(acc_raw[2]*acc_raw[2]));

	 	 	  // Radyan Dereceye cevrilir Derece= 180/3.14 = 57.296  57.296 = 1 / (3.14 / 180)
	 	 	  //angle_roll=arcsin(x/acctotal)
	 	 	  angle_roll_acc = asin((float)acc_raw[0]/acc_total_vector)* -57.296; // x ekseni

	 	 	  //angle_pitch=arcsin(y/acctotal)
	 	 	  angle_pitch_acc = asin((float)acc_raw[1]/acc_total_vector)* 57.296; // y ekseni

	 	 	  //angle_yaw=arcsin(z/acctotal)
	 	 	  angle_yaw_acc = asin((float)acc_raw[2]/acc_total_vector)* 57.296; // z ekseni




	 	 	  if(set_gyro){
	 	 		  //LPF VE HPF Filtre islemleri gerceklestirilir.
	 	 		  // Gyro HPF, acc LPF filteden gectigi icin bu degerler kullanılmıstır
	 	 		  angle_roll = angle_roll_gyro * 0.9996 + angle_roll_acc * 0.0004;
	 	 		  angle_pitch = angle_pitch_gyro * 0.9996 + angle_pitch_acc * 0.0004;

	 	 		}
	 	 	  else
	 	 	  {
	 	 		  angle_pitch = angle_pitch_acc;
	 	 		  set_gyro = true;
	 	 		}

	 	 	  //dongunun 4ms surmesi icin  yazılmıstır
	 	 	  while((HAL_GetTick() - prevtime)*1000 < 4000);
	 	 	  prevtime = HAL_GetTick();
	 	 	  /*****************************************MPU6050 GYRO+ACC SENSOR***********************************************************/


	 	 	          /****************************************RTC******************************************/
	 	 	      HAL_RTC_GetTime(&hrtc , &sTime , RTC_FORMAT_BIN);
	 	 		  HAL_RTC_GetDate(&hrtc , &DateToUpdate , RTC_FORMAT_BIN);

	 	 		  saat = sTime.Hours;
	 	 		  dakika = sTime.Minutes;
	 	 		  saniye = sTime.Seconds;

	 	 		  gun = DateToUpdate.Date;
	 	 		  ay = DateToUpdate.Month;
	 	 		  yil = DateToUpdate.Year;

	 	 		  sprintf(line1, "%d:%d:%d",saat,dakika,saniye);
	 	 		  sprintf(line2, "%d:%d:%d",gun,ay,yil);

	 	 		  HAL_Delay(250);
	 	 		/****************************************RTC******************************************/

       /************************************************ESP8266******************************************************/

/*
	 	 	HAL_UART_Transmit(&huart2, (uint8_t*) "AT\r\n", 100, 100);
	 	 	HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 	 	HAL_Delay(100);


	 	 	HAL_UART_Transmit(&huart2, (uint8_t*) "AT+CWMODE=1\r\n", 100, 100);
	 	 	HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 	 	HAL_Delay(100);


	 	 	HAL_UART_Transmit(&huart2, (uint8_t*) "AT+CIFSR\r\n", 100, 100);
	 	 	HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 	 	HAL_Delay(100); */


	 	 	HAL_UART_Transmit(&huart2, (uint8_t*) "AT+CIPMUX=1\r\n", 100, 100);
	 	 	HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 	 	HAL_Delay(400);


	 	 	HAL_UART_Transmit(&huart2, (uint8_t*) "AT+CIPSERVER=1,80\r\n", 100, 100);
	 	 	HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 	 	HAL_Delay(400);


	 	 	HAL_UART_Transmit(&huart2, (uint8_t*) "AT+CIPSEND=0,31\r\n", 100, 100);
	 	    HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 	 	HAL_Delay(400);
	 	 	sprintf(txdata1,"saat=%3d,dk=%3d,sn=%3d,temp=%3d",saat,dakika,saniye,temp);
	 	 	HAL_UART_Transmit(&huart2, (uint8_t*) txdata1, 100, 100);
	 	 	HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 	 	HAL_Delay(800);


	 	 	HAL_UART_Transmit(&huart2, (uint8_t*) "AT+CIPSEND=0,29\r\n", 100, 100);
	 	 	HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 	 	HAL_Delay(400);
	 	 	sprintf(txdata2,"accX=%3d,accY=%3d,accZ=%3d\r\n",acc_raw[0],acc_raw[1],acc_raw[2]);
	 	 	HAL_UART_Transmit(&huart2, (uint8_t*) txdata2, 100, 100);
	 	 	HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 	 	HAL_Delay(800);


	 		HAL_UART_Transmit(&huart2, (uint8_t*) "AT+CIPSEND=0,32\r\n", 100, 100);
	 		HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 		HAL_Delay(400);
	 		sprintf(txdata3,"gyroX=%3d,gyroY=%3d,gyroZ=%3d\r\n",gyro_raw[0],gyro_raw[1],gyro_raw[2]);
	 		HAL_UART_Transmit(&huart2, (uint8_t*) txdata3, 100, 100);
	 		HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 		HAL_Delay(800);


	 	  /*  HAL_UART_Transmit(&huart2, (uint8_t*) "AT+CIPSEND=0,35\r\n", 100, 100);
	 		HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 		HAL_Delay(400);
	 	    sprintf(txdata4,"angleX=%3d,angleY=%3d\r\n",angle_roll,angle_pitch);
	 		HAL_UART_Transmit(&huart2, (uint8_t*) txdata4, 100, 100);
	 		HAL_UART_Receive(&huart2, (uint8_t*) rxdata, sizeof(rxdata), 100);
	 		HAL_Delay(800); */


	 /************************************************ESP8266******************************************************/


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x21;
  sTime.Minutes = 0x25;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_THURSDAY;
  DateToUpdate.Month = RTC_MONTH_MAY;
  DateToUpdate.Date = 0x12;
  DateToUpdate.Year = 0x21;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

