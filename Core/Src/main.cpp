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
#include "gpio.h"
#include "i2c.h"
#include "stm32_hal_legacy.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_uart.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "3D2.hpp"
#include "oled.h"
#include <cmath>
#include <string>
#define RotationSpeed 0.5

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
Graph2 graph({0, 0, -300}, {128, 64});
Point3<float> points[] = {{-10, 10, 200},  {10, 10, 200},  {10, -10, 200},
                          {-10, -10, 200}, {-10, 10, 220}, {10, 10, 220},
                          {10, -10, 220},  {-10, -10, 220}};
Point3<float> points2[] = {{10, -10, 200},
                           {30, -10, 200},
                           {30, -10, 220},
                           {10, -10, 220},
                           {20, 10, 210}};
Point3<float> front[] = {{-10, 8, 200},  {10, 8, 200},   {-10, 6, 200},
                         {10, 6, 200},   {-10, 4, 200},  {10, 4, 200},
                         {-10, 2, 200},  {10, 2, 200},   {-10, 0, 200},
                         {10, 0, 200},   {-10, -2, 200}, {10, -2, 200},
                         {-10, -4, 200}, {10, -4, 200},  {-10, -6, 200},
                         {10, -6, 200},  {-10, -8, 200}, {10, -8, 200}};
char cmd[1] = "";
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  HAL_UART_Transmit_IT(&huart2, (uint8_t *)cmd, sizeof(cmd));
  float speed = 1;

  switch (cmd[0]) {
  case 'j':
    graph.camera_.pos.y -= speed;
    break;
  case 'k':
    graph.camera_.pos.y += speed;
    break;
  case 'w':
    graph.camera_.pos.z += speed * cos(graph.camera_.rotation.y);
    graph.camera_.pos.x -= speed * sin(graph.camera_.rotation.y);
    break;
  case 'a':
    graph.camera_.pos.z -= speed * sin(graph.camera_.rotation.y);
    graph.camera_.pos.x -= speed * cos(graph.camera_.rotation.y);
    break;
  case 's':
    graph.camera_.pos.z -= speed * cos(graph.camera_.rotation.y);
    graph.camera_.pos.x += speed * sin(graph.camera_.rotation.y);
    break;
  case 'd':
    graph.camera_.pos.z += speed * sin(graph.camera_.rotation.y);
    graph.camera_.pos.x += speed * cos(graph.camera_.rotation.y);
    break;
  case 'q':
    graph.camera_.rotation.y += RotationSpeed * 3.14 / 180;
    break;
  case 'e':
    graph.camera_.rotation.y -= RotationSpeed * 3.14 / 180;
    break;
  case 'z':
    graph.camera_.rotation.x += RotationSpeed * 3.14 / 180;
    break;
  case 'c':
    graph.camera_.rotation.x -= RotationSpeed * 3.14 / 180;
    break;
  case 'u':
    graph.camera_.rotation.z -= RotationSpeed * 3.14 / 180;
    break;
  case 'i':
    graph.camera_.rotation.z += RotationSpeed * 3.14 / 180;
    break;
  case 'r':
    Point3<float> pos_cube = {0, 0, 210};
    float dis_x = (graph.camera_.pos.x - pos_cube.x);
    // float dis_y = (graph.camera_.pos.y - 0);
    float dis_z = (graph.camera_.pos.z - pos_cube.z);

    constexpr float one_deg = RotationSpeed * 3.14 / 180;
    graph.camera_.pos.x =
        pos_cube.x + dis_x * cos(one_deg) - dis_z * sin(one_deg);
    graph.camera_.pos.z =
        pos_cube.z + dis_x * sin(one_deg) + dis_z * cos(one_deg);
    graph.camera_.rotation.y += one_deg;
    break;
  }

  HAL_UART_Receive_IT(&huart2, (uint8_t *)cmd, sizeof(cmd));
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  HAL_Delay(20);
  OLED_Init();
  OLED_Test();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_UART_Transmit_IT(&huart2, (uint8_t *)cmd, sizeof(cmd));

  HAL_UART_Receive_IT(&huart2, (uint8_t *)cmd, sizeof(cmd));
  // graph.camera_.rotation.x -= 10*3.14/180;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    newFrame();

    graph.drawLine3D(points[0], points[1]);
    graph.drawLine3D(points[1], points[2]);
    graph.drawLine3D(points[2], points[3]);
    graph.drawLine3D(points[3], points[0]);

    graph.drawLine3D(points[4], points[5]);
    graph.drawLine3D(points[5], points[6]);
    graph.drawLine3D(points[6], points[7]);
    graph.drawLine3D(points[7], points[4]);

    graph.drawLine3D(points[0], points[4]);
    graph.drawLine3D(points[1], points[5]);
    graph.drawLine3D(points[2], points[6]);
    graph.drawLine3D(points[3], points[7]);

    graph.drawLine3D(points2[0], points2[1]);
    graph.drawLine3D(points2[1], points2[2]);
    graph.drawLine3D(points2[2], points2[3]);
    graph.drawLine3D(points2[3], points2[0]);

    graph.drawLine3D(points2[0], points2[4]);
    graph.drawLine3D(points2[1], points2[4]);
    graph.drawLine3D(points2[2], points2[4]);
    graph.drawLine3D(points2[3], points2[4]);

    for (int i = 0; i <= 16; i += 2) {
      graph.drawLine3D(front[i], front[i + 1]);
    }

    showFrame();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
