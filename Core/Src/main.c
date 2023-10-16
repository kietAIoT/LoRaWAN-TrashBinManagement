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
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void KIET_BlinkLED();
void KIET_ToggleLED();
void KIET_RTC_Init();
void KIET_GPIO_Init();
void KIET_configure_rtc_register();
void KIET_revise();
//void KIET_configure_rtc_register();
void KIET_reset_rtc_register();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	printf("MAIN==>RUNNIG\n");
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
  MX_I2C2_Init();
//  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  KIET_RTC_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	  KIET_ToggleLED();
	  HAL_Delay(100);
	  printf("CHEKC RTC_ DIVH %d, and DIVL: %d\n ", RTC->DIVH ,RTC->DIVL);

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void KIET_RTC_Init(){
	/** @note */

	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);

	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_BKPEN);

	SET_BIT(PWR->CR, PWR_CR_DBP);

	printf("CHEKC POINT -1: %d\n", READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY));
	SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
	uint8_t count = 0;

	while (READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY)==0) {
		count++;
		if (count>100) {
			printf("Have a problem for connecting the External Crystal Clock!");
			break;
		} else;
	}

	SET_BIT(RCC->BDCR, 9);
	CLEAR_BIT(RCC->BDCR, 8);
	SET_BIT(RCC->BDCR, RCC_BDCR_RTCEN);
	printf("CHEKC POINT -1: %d\n", RCC->BDCR);
	/* Bits 9:8 RTCSEL[1:0]: RTC clock source selection
	 * Set by software to select the clock source for the RTC. Once the RTC clock source has been
	 * selected, it cannot be changed anymore unless the Backup domain is reset. The BDRST bit can be used to reset them.
	 * 00: No clock
	 * 01: LSE oscillator clock used as RTC clock
	 * 10: LSI oscillator clock used as RTC clock
	 * 11: HSE oscillator clock divided by 128 used as RTC clock*/



	KIET_configure_rtc_register();
}



void KIET_configure_rtc_register() {
	/* 1. Poll RTOFF, wait until its value goes to ‘1
	 * 2. Set the CNF bit to enter configuration mode
	 * 3. Write to one or more RTC registers
	 * 4. Clear the CNF bit to exit configuration mode
	 * 5. Poll RTOFF, wait until its value goes to ‘1’ to check the end of the write operation*/
	printf("CHEKC POINT 0: %d\n", READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY)); //OKEE ==> VẤn đ�? ở CLOK chưa được kết nối vào
	while (READ_BIT(RTC->CRL, RTC_CRL_RTOFF)==0) {printf("HAL_L1_Check RTOFF\n");}
	SET_BIT(RTC->CRL, RTC_CRL_CNF);
	printf("CHEKC POINT 1: %d\n", READ_BIT(RTC->CRL, RTC_CRL_RTOFF));
	/*Begin for writing to RTC Register - Write one or more RTC register*/
	RTC->PRLH = 0U;
	RTC->PRLL = 0x7FFFU;
	printf("CHEKC POINT 2: %d\n", READ_BIT(RTC->CRL, RTC_CRL_RTOFF));
	RTC->DIVH = 0x0000U;
	RTC->DIVL = 0x8000U;
	printf("CHEKC POINT 3: %d\n", READ_BIT(RTC->CRL, RTC_CRL_RTOFF));
	RTC->CNTH = 0x0000U;
	RTC->CNTL = 0x0000U;

	RTC->ALRH = 0x0000U;
	RTC->ALRL = 0x0004U;
	SET_BIT(RTC->CRH, RTC_CRH_ALRIE);
	SET_BIT(RTC->CRH, RTC_CRH_OWIE);
	//	SET_BIT(RTC->CRH, RTC_CRH_SECIE);
	/*End of writing to RTC register*/
	CLEAR_BIT(RTC->CRL, RTC_CRL_CNF);
	printf("CHEKC POINT 4: %d\n", READ_BIT(RTC->CRL, RTC_CRL_RTOFF));
	while (READ_BIT(RTC->CRL, RTC_CRL_RTOFF)==0) {printf("HAL_L2_Ongoing in other command\n %d",READ_BIT(RTC->CRL, RTC_CRL_RTOFF) );}
	printf("Done configuration RTC\n");

}


void KIET_revise() {
		while (READ_BIT(RTC->CRL, RTC_CRL_RTOFF)==0) {printf("HAL_L1_Check RTOFF\n");}
		SET_BIT(RTC->CRL, RTC_CRL_CNF);
		/*Begin for writing to RTC Register - Write one or more RTC register*/
	//	RTC->DIVH = 0x0000U;
	//	RTC->DIVL = 0x8000U;
		RTC->CNTH = 0x0000U;
		RTC->CNTL = 0x0000U;
		/*End of writing to RTC register*/
		CLEAR_BIT(RTC->CRL, RTC_CRL_CNF);
		while (READ_BIT(RTC->CRL, RTC_CRL_RTOFF)==0) {printf("HAL_L2_Ongoing in other command\n %d",READ_BIT(RTC->CRL, RTC_CRL_RTOFF) );}
}
void KIET_reset_rtc_register() {
	SET_BIT(RCC->BDCR, RCC_BDCR_BDRST);
	HAL_Delay(1);
	CLEAR_BIT(RCC->BDCR, RCC_BDCR_BDRST);
}

void KIET_BlinkLED() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

void KIET_ToggleLED() {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
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