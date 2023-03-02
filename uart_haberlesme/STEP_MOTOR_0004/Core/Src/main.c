/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  *
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
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t pres;
uint32_t count;int duty_cycle = 0,GL_timer_1ms_flag=0,x=1,pulse_sayisi=0,tur_sayisi=0,Start_flag=0;
uint32_t i=0,sayi=0;
int baslangic_sure=0;
//int tur1,tur2,tur3,tur4;
int sayac=0;
int tur_sayisi_flag=0,Gl_step_degeri=800,GL_frekans_degeri;
uint8_t yon=0;
int paket_aldim_flag=0;
int saniye=1;
uint8_t  GL_timer_enable_flag = 0x01;

//**********uart degiskenleri**********
uint8_t myTxData[100]="gelen_veri\r\n";

//uint8_t myRxData[]="gelen_veri\r\n";

//uint8_t myRxData[16]=";5eeeeeeeeeeeL\r\n";
uint8_t myRxData[16]=";8000000000004\r\n";
uint8_t paket_icindeki_data_arrayi[12]={59,59,59,59,59,59,59,59,59,59,59,59};
uint8_t myTx_sayac=0,myRx_sayac=0;
uint8_t rx_flag=0;
uint8_t Gl_step[5],Gl_tur[5],Gl_saniye[5],Gl_frekans[5];
uint8_t Gl_basla_flag=0,Gl_bekle_flag=0;
uint8_t Gl_sag_flag,Gl_sol_flag,Gl_adim_dondurme_flag;
uint8_t sag_flag=0;
uint8_t sol_flag=0;
uint8_t tam_flag=0;
uint8_t sifir,bir,iki,uc;


int  GL_istenen_tur_sayisi=1;
uint8_t motor_dur_flag=0;
int GL_istenen_bekleme_suresi_ms =20;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */



  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */

  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

typedef enum
{
	MOTOR_SAGA_DON,
	MOTOR_SOLA_DON
}motor_yonleri_t;

void stepper_initialize(unsigned int motor_yonu)
{
	HAL_GPIO_WritePin(GPIOA, STEPPER_ENABLE_Pin, GPIO_PIN_RESET);		// step motor enable aktif eder. lowa çekilmeli
	switch (motor_yonu)
	{
		case (unsigned int)MOTOR_SAGA_DON:
		{
			HAL_GPIO_WritePin(GPIOA, DIR_Pin, GPIO_PIN_RESET);
			break;
		}
		case (unsigned int)MOTOR_SOLA_DON:
		{
			HAL_GPIO_WritePin(GPIOA, DIR_Pin, GPIO_PIN_SET);
			break;
		}
		default:
		{
			HAL_GPIO_WritePin(GPIOA, DIR_Pin, GPIO_PIN_SET);
			break;
		}
	}
}




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void timer2_start_stop(int donme_yon)
{

  HAL_TIM_Base_Stop_IT(&htim2);
  HAL_Delay(10);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_Delay(10);
  stepper_initialize(donme_yon);
}

int tur_hesapla(int pulse_sayisi,int step_adim)
{
	int tur_sayisi;
	tur_sayisi=pulse_sayisi/step_adim;
	return tur_sayisi;
}


uint8_t paket_index=0;
uint8_t basladi_flag=0;
uint8_t bitti_flag=0;
uint8_t kontrol_flag=0;
uint8_t toplam;
uint8_t motor_istenen_tur_sayisi;
uint8_t emir_no_boyut=0;
uint8_t data_satir_sayisi[2];
int emir_no_sayac=0;
uint8_t excel_emir_no_array[2];
uint8_t excel_adim_sayisi_array[4];
uint8_t excel_bekleme_suresi_array[2];
uint8_t excel_emir_tekrar_sayisi_array[2];

#define SENKRON_BYTE 59
#define END_BYTE_L	13
#define END_BYTE_H	10
#define VERI_BOYUTU	17
#define PAKET_BOYUTU VERI_BOYUTU+4
typedef struct
{
	uint8_t senkron_byte;
	uint8_t index_no;
	uint8_t veri_bufferi[VERI_BOYUTU];
	uint8_t end_byte_l;
	uint8_t end_byte_h;
}paket_t;
void paket_parser()
{
	for(int i=0; i<VERI_BOYUTU;i++)
	{
			if(myRxData[i]==SENKRON_BYTE)
			{
				basladi_flag=1;
				if(myRxData[14]==END_BYTE_L && myRxData[15]==END_BYTE_H)
				{
					bitti_flag=1;
					paket_icindeki_data_arrayi[0] = myRxData[2];
					paket_icindeki_data_arrayi[1] = myRxData[3];
					paket_icindeki_data_arrayi[2] = myRxData[4];
					paket_icindeki_data_arrayi[3] = myRxData[5];
					paket_icindeki_data_arrayi[4] = myRxData[6];
					paket_icindeki_data_arrayi[5] = myRxData[7];
					paket_icindeki_data_arrayi[6] = myRxData[8];
					paket_icindeki_data_arrayi[7] = myRxData[9];
					paket_icindeki_data_arrayi[8] = myRxData[10];
					paket_icindeki_data_arrayi[9] = myRxData[11];
					paket_icindeki_data_arrayi[10] = myRxData[12];
					paket_icindeki_data_arrayi[11] = myRxData[13];

					if(myRxData[1]==56)//8.paket excel komutlari
					{
						emir_no_sayac=0;
						    for(int i=2;i<=11;i++)
						    {
						    	if(myRxData[i]==48)// myRxDatanýn veri datasi 2-11 arasý 0 ise
						    	{
						    		emir_no_sayac++;
						    	}
						    	if(emir_no_sayac==10)
						    	{
						    		data_satir_sayisi[0]=myRxData[12];//Toplam satýr sayisi yazilir
						    		data_satir_sayisi[1]=myRxData[13];
						    		emir_no_boyut=10*data_satir_sayisi[0]+data_satir_sayisi[1]*1;
						    		myTxData[16]=';'+'8'+'000000000'+'A'+data_satir_sayisi[1]+'\n'+'\r';//paket aldim datasi c# a gonderilir.
						    	}
						    }
						    paket_index=paket_aldim_flag;
							excel_emir_no_array[0+paket_aldim_flag+paket_index]=paket_icindeki_data_arrayi[2];
							excel_emir_no_array[1+paket_aldim_flag+paket_index]=paket_icindeki_data_arrayi[3];
							excel_adim_sayisi_array[0+paket_aldim_flag+paket_index]=paket_icindeki_data_arrayi[4];
							excel_adim_sayisi_array[1+paket_aldim_flag+1+paket_index]=paket_icindeki_data_arrayi[5];
							excel_adim_sayisi_array[2+paket_aldim_flag+2+paket_index]=paket_icindeki_data_arrayi[6];
							excel_adim_sayisi_array[3+paket_aldim_flag+3+paket_index]=paket_icindeki_data_arrayi[7];
							excel_bekleme_suresi_array[0+paket_aldim_flag+paket_index]=paket_icindeki_data_arrayi[8];
							excel_bekleme_suresi_array[1+paket_aldim_flag+1+paket_index]=paket_icindeki_data_arrayi[9];
							excel_emir_tekrar_sayisi_array[0+paket_aldim_flag+2+paket_index]=paket_icindeki_data_arrayi[10];
							excel_emir_tekrar_sayisi_array[1+paket_aldim_flag+3+paket_index]=paket_icindeki_data_arrayi[11];
						    paket_index++;
					}
				}
			}
	}
	basladi_flag=0;
	bitti_flag=0;
}

void array_kontrol_func()
{
	sag_flag=0;
	sol_flag=0;
	tam_flag=0;
	uint8_t sifir_sayac=0;
	    for(int i=0;i<3;i++)
	    {
	    	if(paket_icindeki_data_arrayi[i]==48)//adim flag
	    	{
	    		sifir_sayac=sifir_sayac+1;
	    	}
	    	if(paket_icindeki_data_arrayi[i]==101)//basladi flag
	    	{
	    		sifir_sayac=sifir_sayac+2;
	    	}
	    	if(myRxData[i+1]==48)
	    	{
	    		sifir_sayac=sifir_sayac+3;
	    	}
	    }
	    if(sifir_sayac==3)
	    {
	    	if(paket_icindeki_data_arrayi[3]==84)
	    	{
	    		motor_dur_flag=1;
	    		sag_flag=1;
	    		motor_istenen_tur_sayisi=2;
	    		stepper_initialize(MOTOR_SAGA_DON);
	    		if(tur_sayisi>=motor_istenen_tur_sayisi)
	    		{
	    			myRxData[16]=";5eeeL\r\n";
	    			HAL_GPIO_WritePin(GPIOA, PIN_1_Pin, GPIO_PIN_RESET);
	    			motor_dur_flag=0;
	    			sag_flag=1;
	    		}
	    	}
	    	if(paket_icindeki_data_arrayi[3]==82)
	    	{
	    		motor_dur_flag=1;
	    		 sol_flag=1;
	    		 motor_istenen_tur_sayisi=3;
	    		 stepper_initialize(MOTOR_SOLA_DON);
	    		 if(tur_sayisi>=motor_istenen_tur_sayisi)
	    		 {
	    			 myRxData[16]=";5eeeL\r\n";
	    			 HAL_GPIO_WritePin(GPIOA, PIN_1_Pin, GPIO_PIN_RESET);
	    			 motor_dur_flag=0;
	    			 sol_flag=0;
	    		 }
	    	}
	    	if(paket_icindeki_data_arrayi[3]==76)
	    	{
	    		motor_dur_flag=1;
	    		tam_flag=1;
	    		motor_istenen_tur_sayisi=4;
	    		stepper_initialize(MOTOR_SAGA_DON);
	    		 if(tur_sayisi>=motor_istenen_tur_sayisi)
	    		 {
	    			 myRxData[8]=";50000000SOL\r\n";
	    			 HAL_GPIO_WritePin(GPIOA, PIN_1_Pin, GPIO_PIN_RESET);
	    			 motor_dur_flag=0;
	    			 tam_flag=0;
	    		 }
	    	}
	    }
	    if(sifir_sayac==5)
	    {
	    	if(paket_icindeki_data_arrayi[3]==82)
	    		  {
	    		      basladi_flag=1;
	    		      motor_istenen_tur_sayisi=0;
	    		       stepper_initialize(MOTOR_SAGA_DON);
	    		        myRxData[16]=";50000000SAG\r\n";
	    		    		 HAL_GPIO_WritePin(GPIOA, PIN_1_Pin, GPIO_PIN_RESET);
	    		    		 motor_dur_flag=0;
	    		    		  tam_flag=0;
	    		   }
	    }
	    if(sifir_sayac==9)
	    {
	    	for(int i=0;i<=emir_no_boyut;i++)
	    	{
	    		if(paket_icindeki_data_arrayi[1]==i+1)
	    		{
	    			if(paket_icindeki_data_arrayi[15]==13 && paket_icindeki_data_arrayi[16]==10)
	    			{
		    			paket_aldim_flag++;
		    			for(int i=0;i<=emir_no_boyut;i++)
		    			{
		    				excel_emir_no_array[i];
		    			}
	    			}
	    		}
	    	}
	    }
}


void adim_dondurme_packet(uint8_t son_data,uint8_t paket_flag,uint8_t x)
{
	uint8_t mydata[3]={48,48,48,son_data};
	for(int i=0;i<4;i++)
	{
		if(mydata[i]==paket_icindeki_data_arrayi[i])
		{
			paket_flag=1;
			motor_istenen_tur_sayisi=x;
		}
	}
}



void READ_ADC()
{
     HAL_ADC_Start(&hadc);
	if(HAL_ADC_PollForConversion(&hadc, 100000)==HAL_OK)
	{
		yon = HAL_ADC_GetValue(&hadc);

	}
	HAL_ADC_Stop(&hadc);

}
void bekleme_fonksiyon()
{
	int a  = 0;
	for(int i=0;i<20001;i++)
	{
		a++;
	}

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim2)
		{
			if(GL_timer_enable_flag)
			{
				//tur_sayisi=tur_hesapla(pulse_sayisi,800);
				HAL_GPIO_WritePin(GPIOA, STEPPER_ENABLE_Pin, GPIO_PIN_RESET);
							if(GL_timer_1ms_flag ==0)
							{
			                    GL_timer_1ms_flag = 1;
								HAL_GPIO_WritePin(GPIOA, PIN_1_Pin,GPIO_PIN_SET);
							}
							else if(GL_timer_1ms_flag ==1)
							{
								GL_timer_1ms_flag = 0;
								pulse_sayisi++;
								HAL_GPIO_WritePin(GPIOA, PIN_1_Pin, GPIO_PIN_RESET);

								if(pulse_sayisi>=1600)
								{
									tur_sayisi++;
									pulse_sayisi =0;
								}
								if(tur_sayisi>=GL_istenen_tur_sayisi)
								{
									GL_timer_enable_flag =0;
									tur_sayisi =0;
								}
							}
			}
		}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	HAL_UART_Receive_IT(&huart2,&myRxData,16);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef*UartHandle)
{
	HAL_UART_Receive_IT(&huart2,&myTxData,16);
}

uint8_t GL_motor_is_yapmaya_basladi_flag  =0;
uint8_t motor_is_yap(int istenen_tur_sayisi, int istenen_bekleme_suresi_ms)
{
	if(GL_motor_is_yapmaya_basladi_flag == 0)
	{
		GL_istenen_tur_sayisi = istenen_tur_sayisi;
		GL_istenen_bekleme_suresi_ms = istenen_bekleme_suresi_ms;
		GL_timer_enable_flag = 1;
		GL_motor_is_yapmaya_basladi_flag = 1;
		return  0;
	}
	if((GL_motor_is_yapmaya_basladi_flag==1)&&(!GL_timer_enable_flag))
	{
		HAL_Delay(GL_istenen_bekleme_suresi_ms);
		GL_motor_is_yapmaya_basladi_flag = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//logic and if needed

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
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int motor_istenen_tur_sayisi;

  HAL_TIM_Base_Start_IT(&htim2);
  int motor_istenen_bekleme_suresi_ms;
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);

  while (1)
  {
	      HAL_UART_Receive_IT(&huart2,&myRxData,16);
	  	  HAL_UART_Transmit(&huart2,&myRxData[0],8,10);
          HAL_Delay(500);

	   	  paket_parser();
	      array_kontrol_func();

	  	  if(motor_is_yap(motor_istenen_tur_sayisi,motor_istenen_bekleme_suresi_ms))
	  	  {
	  		 motor_istenen_bekleme_suresi_ms=1000;
	  		  motor_istenen_tur_sayisi=1;
	  	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_8B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 19;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 57600;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|DIR_Pin|STEPPER_ENABLE_Pin|GPIO_PIN_10
                          |PIN_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 DIR_Pin STEPPER_ENABLE_Pin PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|DIR_Pin|STEPPER_ENABLE_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_1_Pin */
  GPIO_InitStruct.Pin = PIN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(PIN_1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
////UART FONKSÝYONLARI//////////






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
