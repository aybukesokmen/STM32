/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t myTxData[100]="gelen_veri\r\n";

//uint8_t myRxData[]="gelen_veri\r\n";

uint8_t myRxData[]="geeeel\r\n";
uint8_t myRxData_gelen[]="";

uint8_t myTx_sayac=0,myRx_sayac=0;
uint8_t rx_flag=0;
uint8_t Gl_step[5],Gl_tur[5],Gl_saniye[5],Gl_frekans[5];
uint8_t Gl_basla_flag=0,Gl_bekle_flag=0,bitti_flag=0;
uint8_t Gl_sag_flag,Gl_sol_flag,Gl_adim_dondurme_flag;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

 //  HAL_UART_Receive_DMA(&huart2,&myTxData,11);

  __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
	 HAL_UART_Receive_IT(&huart2,&myRxData,9);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 // Rx_Flag = 0;
	  //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);

		  	 // HAL_UART_Transmit(&huart2,&myTxData, 13, 10);

		    //HAL_UART_Transmit(&huart2,&myRxData[myRx_sayac],13,10);
	    	HAL_UART_Transmit(&huart2,&myRxData[0],9,10);
	    	myRxData[0] = myRxData[0] ;
	    	myRxData[1] = myRxData[1] ;
	    	myRxData[2] = myRxData[2] ;
	    	myRxData[3] = myRxData[3] ;
	    	myRxData[4] = myRxData[4] ;
	    	myRxData[5] = myRxData[5] ;
	    	myRxData[6] = myRxData[6] ;
		    myRx_sayac++;


	       HAL_Delay(500);
	      // dizi_sinirlama(8);
	   if(myRxData[0]==59)//;
	   {
		   led(0);
		   bekle_flag_fonksiyon();
		   step_flag();
		   frekans_flag();

		   saniye_flag();
		   adim_dondur();
	   }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void bekle_flag_fonksiyon()
{
	if(myRxData[1]==55 )
	{
		if (myRxData[2]==42 && myRxData[3]==42 && myRxData[4]==42 && myRxData[5]==42 && myRxData[6]==42)
		{
			led(7);
			bitme_biti();

		}

	}
}

void bitme_biti()
{
	if(myRxData[7]==13 && myRxData[8]==10)
	{
		bitti_flag=1;

	}
	else{
		bitti_flag=0;
	}
}


void veri_hesapla(uint8_t veri[5])
{
	 for(int i=2;i<7;i++)
	 {
		 veri[i-2]=myRxData[i];
	 }
}
void step_flag()
{
	if(myRxData[1]==50 )
		{
			veri_hesapla(Gl_step);
			led(2);
			bitme_biti();
		}
}
void frekans_flag()
{
	if(myRxData[1]==52 )
	{
		veri_hesapla(Gl_frekans);
		led(4);
		bitme_biti();
	}
}

void led(uint8_t x)
{
	for(int i=0;i<x;i++)
	{
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		HAL_Delay(200);
	}
}

void saniye_flag()
{

	if(myRxData[1]==53 )
			{
				veri_hesapla(Gl_saniye);
				led(5);
				bitme_biti();
	       }
}



void adim_dondur()
{
	if(myRxData[1]==56 )
		{
		      Gl_adim_dondurme_flag=1;

			  veri_hesapla(Gl_tur);

			  if(Gl_tur[5]==82)
			  {
				 Gl_sag_flag=1;
				 led(6);
				  bitme_biti();
			  }
			  else if(Gl_tur[5]==76)
			  {
				  Gl_sol_flag=1;
				  led(6);
				  bitme_biti();
			  }
			  else{
				  Gl_sag_flag=0;
				  Gl_sol_flag=0;
				  Gl_adim_dondurme_flag=0;
				  bitme_biti();
			   }
		}
	else{
		Gl_adim_dondurme_flag=0;
	}

}

uint8_t index_no;



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	HAL_UART_Receive_IT(&huart2,&myRxData,9);


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
