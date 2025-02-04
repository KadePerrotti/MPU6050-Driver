/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "REG_OPTIONS.h"
#include "MPU6050.h"
#include "example.h"
#include "testFunctions.h"
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  MPU6050_REG_READ_TYPE* readReg = MPU6050_REG_READ_STM32;
  MPU6050_BURST_READ_TYPE* burstRead = MPU6050_BURST_READ_STM32;
  MPU6050_REG_WRITE_TYPE* writeReg = MPU6050_REG_WRITE_STM32;


  uint16_t result = init_mpu6050(writeReg);
  
  //setup the low pass filter. 
  writeReg(REG_CONFIG, DLPF_CFG_6 | EXT_SYNC_OFF);

  //select Gyroscope's full scale range
  writeReg(REG_GYRO_CONFIG, GYRO_FS_SEL_250_DPS);   
  
  //select Accelerometer's full scale range
  writeReg(REG_ACCEL_CONFIG, ACCEL_FS_2G);

  //setup the sample rate divider
  writeReg(REG_SMPRT_DIV, SAMPLE_RATE_100Hz);

  read_setup_registers(readReg);

  //enable the fifo
  writeReg(REG_USER_CTRL, FIFO_EN);
  HAL_Delay(100);

  gyro_self_test(readReg, writeReg);
  accel_self_test(readReg, writeReg);
  poll_axes_individually(readReg);

  // //3 gyro axes at 100Hz for 1 sec
  writeReg(REG_FIFO_EN, XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
  fifo_count_test(1000, 100, 3, burstRead, readReg);

  // //3 accel axes at 100Hz for 1 sec
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN);
  fifo_count_test(1000, 100, 3, burstRead, readReg);

  // //6 axes at 100Hz for 2 sec (should fail)
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
  fifo_count_test(2000, 100, 6, burstRead, readReg);

  // //3 accel axes at 100Hz for 2 sec (should fail)
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN);
  fifo_count_test(2000, 100, 3, burstRead, readReg);

  // //6 axes at 100Hz for 350ms
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
  fifo_count_test(350, 100, 6, burstRead, readReg);
  
  //6 axes at 50Hz for 800ms
  writeReg(REG_SMPRT_DIV, SAMPLE_RATE_50Hz);
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
  fifo_count_test(800, 50, 6, burstRead, readReg);

  //6 axes at 200Hz for 200ms
  writeReg(REG_SMPRT_DIV, SAMPLE_RATE_200Hz);
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
  fifo_count_test(200, 200, 6, burstRead, readReg);

  //seems like the fifo doesn't like to work if it gets filled with much more than 600 bytes?
  writeReg(REG_SMPRT_DIV, SAMPLE_RATE_100Hz);
  writeReg(REG_FIFO_EN, 0); //disable fifo for poll axis test
  
  writeReg(REG_FIFO_EN, ACCEL_FIFO_EN | XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN);
  read_fifo_test(500, burstRead, readReg);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
