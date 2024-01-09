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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "u8g2.h"
#include "OLED.h"
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
uint8_t GetKeyStatus(void)
{
    GPIO_PinState state;
    
    state = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    if (state == GPIO_PIN_SET) {
        HAL_Delay(20);
        state = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
        if (state == GPIO_PIN_SET) {
            return 1;
        }
    }
    
    return 0;
}

// 画眼白
void DrawEyesWhite(u8g2_t *u8g2)
{
    u8g2_SetDrawColor(u8g2, 1);
    u8g2_DrawDisc(u8g2, 32, 32, 30, U8G2_DRAW_ALL);
    u8g2_DrawDisc(u8g2, 96, 32, 30, U8G2_DRAW_ALL);
}

// 画瞳孔
// x, y: [-15, +15]
void DrawEyesPupil(u8g2_t *u8g2, int8_t x, int8_t y)
{
    int8_t xPos = 32 + x;
    int8_t yPos = 32 + y;
    
    u8g2_SetDrawColor(u8g2, 0);
    u8g2_DrawDisc(u8g2, xPos, yPos, 15, U8G2_DRAW_ALL);
    u8g2_DrawDisc(u8g2, xPos + 64, yPos, 15, U8G2_DRAW_ALL);
    
    // 瞳孔上的反光
    u8g2_SetDrawColor(u8g2, 1);
    u8g2_DrawDisc(u8g2, xPos + 10, yPos - 10, 3, U8G2_DRAW_ALL);
    u8g2_DrawDisc(u8g2, (xPos + 64) + 10, yPos - 10, 3, U8G2_DRAW_ALL);
    u8g2_DrawFilledEllipse(u8g2, xPos - 8, yPos + 8, 2, 4, U8G2_DRAW_ALL);
    u8g2_DrawFilledEllipse(u8g2, (xPos + 64) - 8, yPos + 8, 2, 4, U8G2_DRAW_ALL);
}

// 画半闭上的眼睛
void DrawClosedEyes(u8g2_t *u8g2, int8_t x, int8_t y)
{
    int8_t xPos = 32 + x;
    int8_t yPos = 32 + y;
    
    u8g2_SetClipWindow(u8g2,0, 32, 127, 63);
    u8g2_SetDrawColor(u8g2, 1);
    u8g2_DrawDisc(u8g2, 32, 32, 30, U8G2_DRAW_ALL);
    u8g2_DrawDisc(u8g2, 96, 32, 30, U8G2_DRAW_ALL);
    
    // 画瞳孔
    u8g2_SetDrawColor(u8g2, 0);
    u8g2_DrawDisc(u8g2, xPos, yPos, 15, U8G2_DRAW_LOWER_RIGHT|U8G2_DRAW_LOWER_LEFT);
    u8g2_DrawDisc(u8g2, xPos + 64, yPos, 15, U8G2_DRAW_LOWER_RIGHT|U8G2_DRAW_LOWER_LEFT);

    // 瞳孔上的反光
    u8g2_SetDrawColor(u8g2, 1);
    u8g2_DrawDisc(u8g2, xPos + 10, yPos - 10, 3, U8G2_DRAW_ALL);
    u8g2_DrawDisc(u8g2, (xPos + 64) + 10, yPos - 10, 3, U8G2_DRAW_ALL);
    u8g2_DrawFilledEllipse(u8g2, xPos - 8, yPos + 8, 2, 4, U8G2_DRAW_ALL);
    u8g2_DrawFilledEllipse(u8g2, (xPos + 64) - 8, yPos + 8, 2, 4, U8G2_DRAW_ALL);
    u8g2_SetMaxClipWindow(u8g2);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  u8g2_t u8g2;
//  char data[32];    
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  u8g2_Init(&u8g2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    u8g2_FirstPage(&u8g2);
    do
    {
//      u8g2_SetFont(&u8g2, u8g2_font_inb16_mr);

//        memset(data, 0, sizeof(data));
//      sprintf(data, "%d", coordinates[0] * );
//      u8g2_DrawStr(&u8g2, 0, 16, data); // x坐标值

//      memset(data, 0, sizeof(data));
//      sprintf(data, "%d", coordinates[1]);
//      u8g2_DrawStr(&u8g2, 0, 32, data); // y坐标值
        int8_t x;
        int8_t y;

        x = coordinates[1] * 15 / 2048;
        y = coordinates[0] * 15 / 2048;
        
      // 按键状态
      if (GetKeyStatus())  {    // Key up       
        if (x * x + y * y < 15 * 15) 
        {
            DrawEyesWhite(&u8g2);
            DrawEyesPupil(&u8g2, x, y);
        }
        else 
        {
            double angle = atan2(y, x);
            x = 15 * cos(angle);
            y = 15 * sin(angle);
            
            DrawEyesWhite(&u8g2);
            DrawEyesPupil(&u8g2, x, y);
        }
      } else {
        if (x * x + y * y < 15 * 15) 
        {
          DrawClosedEyes(&u8g2, x, y);
        }
        else 
        {
            double angle = atan2(y, x);
            x = 15 * cos(angle);
            y = 15 * sin(angle);
            
          DrawClosedEyes(&u8g2, x, y);
        }
      }
    } while (u8g2_NextPage(&u8g2));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    
    // 再次启动DMA
    ADC_StartDMA();    
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
