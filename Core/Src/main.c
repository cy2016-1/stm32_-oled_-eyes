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
#include "dma.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "u8g2.h"
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI        3.1415926535    
#define START_X     10
#define END_X       118
#define SCOPE_Y     30    


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
u8g2_t u8g2;
int8_t data[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void draw(u8g2_t *u8g2);
void draw_x_axis(void);
void draw_y_axis(void);
void update_data(void);
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
    uint8_t i;
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
  /* USER CODE BEGIN 2 */
  u8g2_Init(&u8g2);
  memset(data, 0xFF, sizeof(data));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    u8g2_FirstPage(&u8g2);
    do
    {
      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
      update_data();
      u8g2_SetFont(&u8g2, u8g2_font_squeezed_r6_tr);

      // 画坐标轴
      draw_x_axis();
      draw_y_axis();

      // 以data为Y轴坐标画线
      for (i = 0; i < 128 - 1; i++)
      {
        if (data[i] >= 0)
        {
          u8g2_DrawLine(&u8g2, i, data[i], i + 1, data[i + 1]);
        }
      }

    } while (u8g2_NextPage(&u8g2));
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
}

/* USER CODE BEGIN 4 */
/*官方logo的Demo*/
void draw(u8g2_t *u8g2)
{
    u8g2_SetFontMode(u8g2, 1); /*字体模式选择*/
    u8g2_SetFontDirection(u8g2, 0); /*字体方向选择*/
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf); /*字库选择*/
    u8g2_DrawStr(u8g2, 0, 20, "U");
    
    u8g2_SetFontDirection(u8g2, 1);
    u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
    u8g2_DrawStr(u8g2, 21,8,"8");
        
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 51,30,"g");
    u8g2_DrawStr(u8g2, 67,30,"\xb2");
    
    u8g2_DrawHLine(u8g2, 2, 35, 47);
    u8g2_DrawHLine(u8g2, 3, 36, 47);
    u8g2_DrawVLine(u8g2, 45, 32, 12);
    u8g2_DrawVLine(u8g2, 46, 33, 12);
  
    u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
    u8g2_DrawStr(u8g2, 1,54,"github.com/olikraus/u8g2");
}

void draw_h_arrow(uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1)
{   
    u8g2_DrawLine(&u8g2, X0, Y0, X1, Y1);
    u8g2_DrawLine(&u8g2, X1 - 5, Y1 - 3, X1, Y1);
    u8g2_DrawLine(&u8g2, X1 - 5, Y1 + 3, X1, Y1);

}

void draw_v_arrow(uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1)
{
    u8g2_DrawLine(&u8g2, X0, Y0, X1, Y1);
    u8g2_DrawLine(&u8g2, X1 - 3, Y1 + 5, X1, Y1);
    u8g2_DrawLine(&u8g2, X1 + 3, Y1 + 5, X1, Y1);
}

// 画x轴
void draw_x_axis(void)
{
    int i;
    
    draw_h_arrow(0, 32, 127, 32);   
    
    u8g2_DrawStr(&u8g2, START_X - 4, 40, "0");
    
    for (i = 1; i <= 3; i++) {
        char buf[3] = {'0', '0', '\0'};
        
        u8g2_DrawVLine(&u8g2, START_X + 30 * i, 29, 3); // 刻度
        buf[0] = '0' + i * 3;
        u8g2_DrawStr(&u8g2, START_X + 30 * i - 2, 40, buf); // 值
    }
}

// 画y轴
void draw_y_axis(void)
{
    int i;
    
    draw_v_arrow(START_X, 63, START_X, 0);  
    
    for (i = 1; i <= 2; i++) {
        char buf[] = {'0', '0', '\0'};
        
        u8g2_DrawHLine(&u8g2, START_X - 3, 32 - 13 * i + 4 , 3);
        buf[0] = '0' + i;
        u8g2_DrawStr(&u8g2, START_X + 2, 32 - 13 * i + 7, buf);
    }
    
    for (i = 1; i <= 2; i++) {
        char buf[] = {'-', '1', '0', '\0'};
        
        u8g2_DrawHLine(&u8g2, START_X - 3, 32 + 13 * i - 4 , 3);    // 刻度
        buf[1] = '0' + i;
        u8g2_DrawStr(&u8g2, START_X + 2, 32 + 13 * i, buf); // 值
    }
}

// 更新数据，这里使用正弦函数生成新数据
void update_data(void)
{
    static double x = 0;
    
    // 将角度转换为弧度  
    double radians = x * M_PI / 180.0;
    
    // 计算sin函数的值  
    double value = sin(radians);  
    value = SCOPE_Y * (1 - value);
    
    // 把data整体前移1
    memmove(data, data + 1, sizeof(data) - sizeof(data[0]));
    data[127] = value;
    
    // 更新下一次的x值
    x += 360.0 / (END_X - START_X);  
    if (x > 360) {
        x -= 360;
    }
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
