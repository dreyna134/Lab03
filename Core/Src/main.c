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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();

	
	// CONFIG TIMERS
	// Enable TIM2 and TIM3 Periph on RCC
	RCC->APB1ENR |= (1 << 0) | (1 << 1);
	
	// Config timer 2 to trigger UEV (update event) at 4 Hz
	// 4 Hz = 250ms period
	uint16_t PSCVal = 7999; // down to 1000Hz (1 ms)
	uint32_t ARRVal = 250; // count 250 (250ms)
	TIM2->PSC = PSCVal;
	TIM2->ARR = ARRVal;
	
	// Config timer 3 to UEV period related to 800Hz
	uint16_t PSCVal_tim3 = 999; // get down to 8000Hz (0.125ms)
	uint32_t ARRVal_tim3 = 10; // 10 counts -> 1.25ms
	TIM3->PSC = PSCVal_tim3;
	TIM3->ARR = ARRVal_tim3;
	
	/* TIM3 PWM CONFIG */
	// Timer 3 -> Config CCMR register to configure PWM mode
	  // Set to output mode (channels 1 and 2)
	TIM3->CCMR1 &= ~((1 << 9) | (1 << 8)); // CC2S to "00", output mode
	TIM3->CCMR1 &= ~((1 << 1) | (1 << 0)); // CC1S to "00"
	
	  // Set OC1M so channel 1 is in PWM Mode 2
	TIM3->CCMR1 |= (1 << 6) | (1 << 5) | (1 << 4); // OC1M to "111"
	
	  // Set OC2M so channel 2 is in PWM Mode 1
	TIM3->CCMR1 |= (1 << 14) | (1 << 13);
	TIM3->CCMR1 &= ~(1 << 12); // OC2M to "110"
	
	  // Enable the output compare preload for both channels
	TIM3->CCMR1 |= (1 << 3) | (1 << 11); // OCP1E and OCP2E to "1"
	
	// Timer 3 -> Set output enable bits for ch1&2 in CCER reg
	TIM3->CCER |= (1 << 0); // CC1E to "1"
	TIM3->CCER |= (1 << 4); // CC2E to "1"
	
	// Timer 3 -> Set capture/compare (CCRx) for both ch to 20% of ARR value
	TIM3->CCR1 = 2; //ARRVal_tim3(10) * 0.2
	TIM3->CCR2 = 2;
	/* END OF TIM3 PWM CONFIG */
	
	
	// Config timer 2 to trigger UEV event
	TIM2->DIER |= (1 << 0);
	
	// Enable/start timer 2
	TIM2->CR1 |= (1 << 0);
	// Enable/start timer 3
	TIM3->CR1 |= (1 << 0);
	
	NVIC_EnableIRQ(TIM2_IRQn);
	
	
	RCC->AHBENR |= (1 << 19); // Enable peripheral clock for port C (19th bit is set)
	// Enable LED pins PC9 and PC8 
	GPIOC->MODER = 0;
	GPIOC->MODER |= (1 << 18) | (1 << 16);
	
	/* CONFIG PC6 AND PC7 TO AF MODE */
	// Set pins PC6 and PC7 to Alternate-function mode
	// for TIM3_CH1 and TIM3_CH2
	GPIOC->MODER |= (1 << 15) | (1 << 13); // Set PC6&7 to "10" AF
	
	// Set AFR register to set appropriate alternate function
	// Set Port C, Pins 6 and 7 to "0000" --> AF0
	GPIOC->AFR[0] &= ~((1 << 31) | (1 << 30) | (1 << 29) | (1 << 28));
	GPIOC->AFR[0] &= ~((1 << 27) | (1 << 26) | (1 << 25) | (1 << 24));
	
	/* END OF PC6 and PC7 AF CONFIG */
	
	// Set pins PC9 and PC8 to 
	GPIOC->OTYPER &= ~((1 << 8) | (1 << 9));
	GPIOC->OTYPER &= ~((1 << 7) | (1 << 6));
	
	// Set pins to low speed in OSPEEDR
	GPIOC->OSPEEDR = 0; // Set to clear all bits such that pins are set to 00, low speed
	
	// Set pins to no pull-up/down resistors in the PUPDR reg
	GPIOC->PUPDR = 0; // Set clear all bits such that pins are set to 00, no pull-up, pull-down
	
	GPIOC->ODR |= (1 << 9); // Set ODR9 to be high
	GPIOC->ODR &= ~(1 << 8); // Set ODR8 to be low
	

  /* Infinite loop */
  while (1)
  {
		
  }
}

/*
TIMER 2 INTERUPT HANDLER
*/
void TIM2_IRQHandler(void)
{
	// Invert LEDs 8 and 9
	GPIOC->ODR ^= (1 << 8) | (1 << 9);
	// Clear status flag
	TIM2->SR &= ~(1 << 0); // Clear bit 0, Update interupt flag
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
