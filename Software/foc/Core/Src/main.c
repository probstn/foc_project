/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "foc.h"
#include <string.h>
#include <inttypes.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG 1

q31_t demo;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

Logger buffer;
Logger cache;

typedef struct
{
  volatile uint32_t last_time;
  volatile float omega;
  volatile float last_angle;
} HALL;
HALL shall;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (!shall.last_time)
  {
    // first time
    shall.last_time = TIM3->CNT;
    return;
  }
  uint8_t hall_state = (HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin) << 2) |
                       (HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) << 1) |
                       (HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) << 0);
  float angle = hall_to_rad[hall_state];
  shall.last_angle = angle;

  uint32_t now_ticks = TIM3->CNT;
  float dt = (uint16_t)(now_ticks - shall.last_time) * HALL_TICK;
  shall.last_time = now_ticks;
  float omega = 1.0472 / dt; // rad/s
  shall.omega = omega;
}

float getAngle()
{
  uint32_t now_ticks = TIM3->CNT;
  float dt = (uint16_t)(now_ticks - shall.last_time) * HALL_TICK;
  float theta = buffer.theta = /*shall.last_angle + */ shall.omega * dt; // rad
  return 0;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_SET);
    /* GET PHASE CURRENTS AND VBAT */
    /*
    // read values directly from registers to save 2.3us
    
    uint32_t adc_value1 = buffer.adc1 = hadc->Instance->JDR1; // HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1)
    uint32_t adc_value2 = buffer.adc2 = hadc->Instance->JDR2; // HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2)
    uint32_t adc_value3 = buffer.adc3 = hadc->Instance->JDR3; // HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3)
    uint32_t adc_value4 = hadc->Instance->JDR4;               // HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4)
    // instead of /24.0f we *1/24.0f to save 5us
    float curr_a = ((int32_t)adc_value1 - 1982.94948f) * 0.043365131f;
    // float curr_b = ((int32_t)adc_value2 - 1978.51862f) * 0.045875989f;
    float curr_c = ((int32_t)adc_value3 - 1980.93166f) * 0.044918113f;
    float curr_b = (curr_a + curr_c) * -1.0f;
    
    buffer.curr_a = curr_a;
    buffer.curr_b = curr_b;
    buffer.curr_c = curr_c;
    
    float vbat = buffer.vbat = adc_value4 * 0.025f; //*1/40 instead of /40 to save time
    
    float theta = getAngle();
    // 6us until here
    
    // foc_update(10.0, curr_a, curr_b, curr_c, theta);
    
    */
    openloop(20);
    
    HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_RESET);
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
  // Enable TRC (Trace)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  // Reset the cycle counter
  DWT->CYCCNT = 0;
  // Enable the cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  foc_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    cache = buffer;
    if (LOG)
    {
      /*
      printf("%.2f,", cache.curr_a);
      printf("%.2f,", cache.curr_b);
      printf("%.2f,", cache.curr_c);
      printf("%lu,", cache.va);
      printf("%lu,", cache.vb);
      printf("%lu,", cache.vc);
      printf("%.2f,", cache.alpha);
      printf("%.2f,", cache.beta);
      printf("%.2f,", cache.vbat);
      */
      printf("%.2f", cache.theta);
      printf("\n");
    }
    /*
    printf("%.2f,", cache.curr_a);
    printf("%.2f,", cache.curr_b);
    printf("%.2f\n", cache.curr_c);
    printf("%lu,", cache.adc1);
    printf("%lu,", cache.adc2);
    printf("%lu\n", cache.adc3);
    */

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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  for (int i = 0; i < len; i++)
  {
    ITM_SendChar((*ptr++)); // send each character via ITM
  }
  return len;
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
#ifdef USE_FULL_ASSERT
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
