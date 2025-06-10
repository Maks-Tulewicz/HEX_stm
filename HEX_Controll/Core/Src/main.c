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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "pca9685.h"
#include "hexapod_kinematics.h"
#include "test_positions.h"
#include "step_functions.h"
#include "tripod_gait.h"
#include "bipedal_gait.h"
#include "wave_gait.h"

#include <stdio.h>

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

PCA9685_Handle_t pca1, pca2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setAllto90(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2)
{
  // Test wszystkich bioder na 90° (środek przedziału)
  PCA9685_SetServoAngle(pca1, 0, 90.0f); // Noga 1 HIP
  PCA9685_SetServoAngle(pca2, 0, 90.0f); // Noga 2 HIP
  PCA9685_SetServoAngle(pca1, 3, 90.0f); // Noga 3 HIP
  PCA9685_SetServoAngle(pca2, 3, 90.0f); // Noga 4 HIP
  PCA9685_SetServoAngle(pca1, 6, 90.0f); // Noga 5 HIP
  PCA9685_SetServoAngle(pca2, 6, 90.0f); // Noga 6 HIP

  HAL_Delay(1000); // Czekaj 1 sekundę, aby zobaczyć pozycje

  PCA9685_SetServoAngle(pca1, 1, 90.0f); // Noga 1 HIP
  PCA9685_SetServoAngle(pca2, 1, 90.0f); // Noga 2 HIP
  PCA9685_SetServoAngle(pca1, 4, 90.0f); // Noga 3 HIP
  PCA9685_SetServoAngle(pca2, 4, 90.0f); // Noga 4 HIP
  PCA9685_SetServoAngle(pca1, 7, 90.0f); // Noga 5 HIP
  PCA9685_SetServoAngle(pca2, 7, 90.0f); // Noga 6 HIP

  HAL_Delay(1000); // Czekaj 1 sekundę, aby zobaczyć pozycje

  PCA9685_SetServoAngle(pca1, 2, 90.0f); // Noga 1 HIP
  PCA9685_SetServoAngle(pca2, 2, 90.0f); // Noga 2 HIP
  PCA9685_SetServoAngle(pca1, 5, 90.0f); // Noga 3 HIP
  PCA9685_SetServoAngle(pca2, 5, 90.0f); // Noga 4 HIP
  PCA9685_SetServoAngle(pca1, 8, 90.0f); // Noga 5 HIP
  PCA9685_SetServoAngle(pca2, 8, 90.0f); // Noga 6 HIP
}

void testStanding(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2)
{
  // Test pozycji stojącej
  PCA9685_SetServoAngle(pca1, 0, 90.0f); // Noga 1 HIP
  PCA9685_SetServoAngle(pca2, 0, 90.0f); // Noga 2 HIP
  PCA9685_SetServoAngle(pca1, 3, 90.0f); // Noga 3 HIP
  PCA9685_SetServoAngle(pca2, 3, 90.0f); // Noga 4 HIP
  PCA9685_SetServoAngle(pca1, 6, 90.0f); // Noga 5 HIP
  PCA9685_SetServoAngle(pca2, 6, 90.0f); // Noga 6 HIP

  HAL_Delay(1000); // Czekaj 1 sekundę, aby zobaczyć pozycje

  PCA9685_SetServoAngle(pca1, 1, 60.0f); // Noga 1 HIP
  PCA9685_SetServoAngle(pca2, 1, 60.0f); // Noga 2 HIP
  PCA9685_SetServoAngle(pca1, 4, 60.0f); // Noga 3 HIP
  PCA9685_SetServoAngle(pca2, 4, 60.0f); // Noga 4 HIP
  PCA9685_SetServoAngle(pca1, 7, 60.0f); // Noga 5 HIP
  PCA9685_SetServoAngle(pca2, 7, 60.0f); // Noga 6 HIP

  HAL_Delay(1000); // Czekaj 1 sekundę, aby zobaczyć pozycje

  PCA9685_SetServoAngle(pca1, 2, 5.0f); // Noga 1 HIP
  PCA9685_SetServoAngle(pca2, 2, 5.0f); // Noga 2 HIP
  PCA9685_SetServoAngle(pca1, 5, 5.0f); // Noga 3 HIP
  PCA9685_SetServoAngle(pca2, 5, 5.0f); // Noga 4 HIP
  PCA9685_SetServoAngle(pca1, 8, 5.0f); // Noga 5 HIP
  PCA9685_SetServoAngle(pca2, 8, 5.0f); // Noga 6 HIP
}

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
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  if (!PCA9685_Init(&pca1, &hi2c1, PCA9685_ADDRESS_1))
  {
    while (1)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED to indicate error
      HAL_Delay(50);
    }
  }

  if (!PCA9685_Init(&pca2, &hi2c2, PCA9685_ADDRESS_1))
  {
    while (1)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED to indicate error
      HAL_Delay(50);
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // testBasicPositions(&pca1, &pca2);

    setAllto90(&pca1, &pca2);   // Ustaw wszystkie serwa na 90°
    HAL_Delay(1000);            // Czekaj 1 sekundę, aby zobaczyć pozycje
    testStanding(&pca1, &pca2); // Test pozycji stojącej
    HAL_Delay(15000);           // Czekaj 1 sekundę, aby zobaczyć pozycje

    // tripodGaitWalk(&pca1, &pca2, TRIPOD_FORWARD, 5);
    // bipedalGaitWalk(&pca1, &pca2, BIPEDAL_FORWARD, 3);
    // waveGaitWalk(&pca1, &pca2, WAVE_FORWARD, 3);

    HAL_Delay(15000); // Czekaj 1 sekundę, aby zobaczyć pozycje

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
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
  /* User can add his own implementation to reporte HAL error return state */
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
