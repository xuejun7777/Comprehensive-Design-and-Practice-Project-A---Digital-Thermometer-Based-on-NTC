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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TSP_SHORT_CIRCUIT_THRESHOLD 	20U			/*!<NTC short-circuit */
#define TSP_OPEN_CIRCUIT_THRESHOLD 		4090U		/*!<NTC open-circuit */
#define LED_SEG_REFRESH_PERIOD		5
#define TSP_NTC_B_VALUE   			3950U 			/*!<NTC B value */
#define TSP_NTC_RATED_TEMP          25U   			/*!<NTC rated temperature(25¡æ) */
#define TSP_NTC_RATED_RES           10000.0F 		/*!<NTC rated resistance 10K*/
#define TSP_NTC_KELVIN_VALUE		273.15F			/*!<NTC kelvin value */
#define TSP_NTC_T2_VALUE			TSP_NTC_KELVIN_VALUE + TSP_NTC_RATED_TEMP
#define TSP_NTC_ERROR_VALUE			0.5F 			/*!<NTC error,log replace of ln, need to add the error */
#define TSP_ADC_FULL_SCALE 		    4095.0F	        /*!<ADC scale*/
#define TSP_ADC_VOLTAGE_REF 	    3.28F	        /*!<Voltage reference*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t adc_convert_cplt;
uint32_t stuid_cplt = 0;
float volt;
float resistant;
float tc;
float tc2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t ch;
uint8_t ch_r;
__IO uint8_t dig_array[4]={0x01, 0x05, 0x01, 0x02};
extern void select_digit(uint8_t dig);
extern void write_segment(uint8_t seg);
extern void DisplayDigitFLOAT(uint32_t num);
extern void DisplayCallback(void);
static float tsp_ResistanceToTemperature( uint16_t adc_val );

const uint8_t seg_table[16] = {
	0x3f,		//"0"
	0x06,		//"1"
	0x5b,		//"2"
	0x4f,		//"3"
	0x66,		//"4"
	0x6d,		//"5"
	0x7d,		//"6"
	0x07,		//"7"
	0x7f,		//"8"
	0x6f,		//"9"
	0x77,		//"A"
	0x7c,		//"b"
	0x39,		//"C"
	0x5e,		//"d"
	0x79,		//"E"
	0x71		//"F"
};

//2151936
const uint32_t StuID[5] = {
	2151, 1519,5193,1936, 9360
};
static const uint16_t NTC_adc_table[] = 
{
	3996, 3988, 3981, 3972, 3964, 3955, 3945, 3935, 3924, 3912, 	
	3900, 3887, 3874, 3860, 3845, 3830, 3813, 3796, 3778, 3760, 	
	3740, 3720, 3698, 3676, 3653, 3629, 3604, 3578, 3551, 3524, 	
	3495, 3465, 3435, 3403, 3371, 3337, 3303, 3267, 3231, 3194, 	
	3156, 3118, 3078, 3038, 2997, 2955, 2913, 2870, 2826, 2782, 	
	2738, 2693, 2648, 2602, 2556, 2510, 2464, 2417, 2371, 2324, 	
	2278, 2231, 2185, 2139, 2093, 2048, 2002, 1957, 1913, 1868, 	
	1825, 1781, 1739, 1697, 1655, 1614, 1574, 1534, 1495, 1456, 	
	1419, 1382, 1346, 1310, 1275, 1241, 1208, 1175, 1143, 1112, 	
	1081, 1052, 1023, 994, 967, 940, 914, 888, 863, 839, 	
	815, 792, 770, 748, 727, 707, 687, 668, 649, 631, 	
	613, 596, 579, 563, 547, 532, 517, 502, 488, 475, 	
	462, 449, 436, 424, 413, 401, 390, 380, 369, 359, 	
	350, 340, 331, 322, 314, 305, 297, 289, 282, 274, 	
	267, 260, 253, 247, 240, 234, 228, 222, 217, 211, 	
	206, 201, 196, 191, 186, 181, 177, 173, 168, 164, 	
	160
};

static const uint16_t NTC_temperature_table[] = 
{
	-40, -39, -38, -37, -36, -35, -34, -33, -32, -31, 	
	-30, -29, -28, -27, -26, -25, -24, -23, -22, -21, 	
	-20, -19, -18, -17, -16, -15, -14, -13, -12, -11, 	
	-10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 	
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 	
	10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 	
	20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 	
	30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 	
	40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 	
	50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 	
	60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 	
	70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 	
	80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 	
	90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 	
	100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 	
	110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 	
	120
};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double tsp_ln(double a)
{
	int N = 15;
	int k , nk;
	double x, xx , y;
	x = ( a - 1 ) / ( a + 1 );
	xx = x * x;
	nk = 2 * N + 1;
	y = 1.0 / nk;
	for( k = N; k > 0; k-- )
	{
		nk = nk - 2;
		y = ( 1.0 / nk ) + ( xx * y );
	}

	return ( 2.0 * x * y );
}
void DisplayCallback(void)
{
	uint8_t dig;
	uint32_t tick = HAL_GetTick();

	if((tick%LED_SEG_REFRESH_PERIOD) == 0)
	{
		dig = (tick/LED_SEG_REFRESH_PERIOD)%4;
		select_digit(dig);

		if(stuid_cplt <10 && stuid_cplt % 5 == 4 && dig == 3)		//digit is off
		{
			write_segment(0x00);			
		}
		else if((dig_array[dig]&0x80) != 0)		//display minus
		{
			write_segment(0x40);	
		}
		else if(dig == 1 && stuid_cplt > 9)		//add dot
		{
			write_segment(seg_table[dig_array[3-dig]&0x0f] | 0x80);
		}
		else
		{
			write_segment(seg_table[dig_array[3-dig]&0x0f]);
		}

	}
}


void select_digit(uint8_t dig)
{
	switch(dig)
	{
		case 0:
			HAL_GPIO_WritePin(GPIOD, LED_CC0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC3_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOD, LED_CC0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC3_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD, LED_CC0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, LED_CC3_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOD, LED_CC0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, LED_CC3_Pin, GPIO_PIN_SET);
			break;
		default:
			break;
	}
}


void write_segment(uint8_t seg)
{
	HAL_GPIO_WritePin(GPIOB, seg, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, ~seg, GPIO_PIN_RESET);
}



void DisplayDigitFLOAT(uint32_t num)
{
	uint32_t tmp;
	if(num > 9999)
	{
		for(uint8_t i=0;i<8;i++)
			dig_array[i] = 0x0f;
	}
	else
	{
		tmp = num;
		for(uint8_t i=0;i<4;i++)
		{
			if((tmp == 0)&&(i!=0))
			{
				dig_array[i] = 0xff;
			}
			else
			{
				dig_array[i] = tmp%10;			
			}
			tmp = tmp/10;
		}
			
	}
}


static float tsp_BinaryTableSearch( uint16_t adc_val )
{
	uint16_t start = 0U, end = 0U, mid = 0U;

	/* Get the arry length */
	end = ( sizeof( NTC_adc_table )/ sizeof( NTC_adc_table[0] ) ) - 1U;

	/* Data anomaly judgment */
	if( adc_val <= TSP_SHORT_CIRCUIT_THRESHOLD )
	{
		return 1.0F;
	}
	else if( adc_val >= TSP_OPEN_CIRCUIT_THRESHOLD )
	{
		return 2.0F;
	}
	else if( adc_val > NTC_adc_table[0] )
	{
		return 3.0F;
	}
	else if( adc_val < NTC_adc_table[end - 1U] )
	{
		return 4.0F;
	}
	else
	{
		/* MISRA-C coding rules */
	}

	while ( start <= end )
	{
		/* Get the mid value */
		mid = (start + end) >> 1; 

		/* Just find */
		if( adc_val ==  NTC_adc_table[mid] )
		{
			break;
		}
		/* Right in between two temperature points */
		if( ( adc_val < NTC_adc_table[mid] ) && ( adc_val > NTC_adc_table[mid+1U] ) )
		{
			break;
		}

		/* The current AD value less than the middle of the array indicates 
		 * the second half of the number to look for 
		 */
		if( adc_val < NTC_adc_table[mid] )
		{
			start = mid + 1U; 
		}
		/* The current AD value greater than the middle of the array indicates 
		 * that the number to be found is in the first half 
		 */
		else if( adc_val > NTC_adc_table[mid] )
		{
			end = mid - 1U;
		}
		else
		{
			/* MISRA-C coding rules */
		}
	}

 	return ( NTC_temperature_table[mid] + (float)( NTC_adc_table[mid] - adc_val ) * (float)( NTC_temperature_table[mid+1] - NTC_temperature_table[mid] ) / (float)( NTC_adc_table[mid]-NTC_adc_table[mid+1] ) );
	
	// return (mid-40) +  (float)(NTC_adc_table[mid] -key)/(float)(NTC_adc_table[mid]-NTC_adc_table[mid+1]);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint16_t value;
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
	{
		value = HAL_ADC_GetValue(&hadc1);
		//volt = value *3.3/4096;
		//resistant = 10000 / (3.3 / volt - 1);
		//tc = tsp_BinaryTableSearch(value);
		tc = tsp_ResistanceToTemperature(value);
		if (++stuid_cplt > 9) {
			adc_convert_cplt = 1;
		}
		else {
			int i = stuid_cplt % 5;
			DisplayDigitFLOAT(StuID[i]);
		}
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
	char txbuf[32];
	int len;
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_IT(&hadc1); 
	HAL_TIM_Base_Start(&htim3); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
	if(adc_convert_cplt == 1)
	{
		memset(txbuf,0,sizeof(txbuf));

		len = sprintf(txbuf,"temperature=%1.2f\n",tc);
		if(len > 0)
			HAL_UART_Transmit_IT(&huart1,(uint8_t*)txbuf,len);
		
		DisplayDigitFLOAT(roundf(tc*100.0f)/100.0f * 100);
		adc_convert_cplt = 0;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static float tsp_ResistanceToTemperature( uint16_t adc_val )
{
	float B_value  = TSP_NTC_B_VALUE;
	float Rp_value = TSP_NTC_RATED_RES;
	float T1 = 0.0F;
	float T2 = TSP_NTC_T2_VALUE;
	float Rt_value = 0.0F;
	float volt_val = 0.0F;

	/* Data anomaly judgment */
	if( adc_val <= TSP_SHORT_CIRCUIT_THRESHOLD )
	{
		return 1.0F;
	}
	else if( adc_val >= TSP_OPEN_CIRCUIT_THRESHOLD )
	{
		return 2.0F;
	}
	else
	{
		/* MISRA-C coding rules */
	}

	/* Convert to voltage */
	volt_val = ((float) adc_val * TSP_ADC_VOLTAGE_REF) / TSP_ADC_FULL_SCALE;

	/* Get the NTC resistance value */
	Rt_value = volt_val / ( ( TSP_ADC_VOLTAGE_REF - volt_val ) / TSP_NTC_RATED_RES );
	
	/* Operational formula */
	//T1 = ( 1.0F / ( ( log( Rt_value / Rp_value ) / B_value ) + ( 1.0F / T2 ) ) );

	/* Operational formula */
	T1 = ( 1.0F / ( ( tsp_ln( Rt_value / Rp_value ) / B_value ) + ( 1.0F / T2 ) ) );

	/* Kelvin value convert to degree celsius value */
	T1 = T1 - TSP_NTC_KELVIN_VALUE;

	return ( T1 );
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
