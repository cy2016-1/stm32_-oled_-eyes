#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "u8g2.h"
#include "u8x8.h"
#include "Delay.h"
#include "i2c.h"

// 硬件I2C
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static uint8_t buffer[1024];
    static uint8_t buf_idx;
    uint8_t *data;
    
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
        data = (uint8_t *)arg_ptr;
        while (arg_int > 0)
        {
            buffer[buf_idx++] = *data;
            data++;
            arg_int--;
        }
      break;
      
    case U8X8_MSG_BYTE_INIT:
      MX_I2C1_Init();
      break;
    
    case U8X8_MSG_BYTE_SET_DC:
      break;
    
    case U8X8_MSG_BYTE_START_TRANSFER:
        buf_idx = 0;
      break;
    
    case U8X8_MSG_BYTE_END_TRANSFER:
      while (HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READY)  {}
      while (HAL_I2C_Master_Transmit(&hi2c1,u8x8_GetI2CAddress(u8x8),buffer,buf_idx, 10)!= HAL_OK)
      {
        if (HAL_I2C_GetError(&hi2c1)!= HAL_I2C_ERROR_AF)
        {
          Error_Handler();
        }
      }        
      break;
    
    default:
      return 0;
  }
  return 1;
}

void delay_us(uint32_t time)
{
    uint32_t i = 8 * time;
    while (i--)
        ;
}

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      /* only support for software I2C*/
      break;
    case U8X8_MSG_DELAY_NANO:
      /* not required for SW I2C */
      break;
    
    case U8X8_MSG_DELAY_10MICRO:
      /* not used at the moment */
      break;
    
    case U8X8_MSG_DELAY_100NANO:
      /* not used at the moment */
      break;
   
    case U8X8_MSG_DELAY_MILLI:
      HAL_Delay(1);
      break;
    case U8X8_MSG_DELAY_I2C:
      /* arg_int is 1 or 4: 100KHz (5us) or 400KHz (1.25us) */
      //delay_us(5);
      break;
    
    case U8X8_MSG_GPIO_I2C_CLOCK:
      break;
    case U8X8_MSG_GPIO_I2C_DATA:
      break;
    case U8X8_MSG_GPIO_MENU_SELECT:
      u8x8_SetGPIOResult(u8x8, 0);
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
      u8x8_SetGPIOResult(u8x8, 0);
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
      u8x8_SetGPIOResult(u8x8, 0);
      break;
    case U8X8_MSG_GPIO_MENU_HOME:
      u8x8_SetGPIOResult(u8x8, 0);
      break;
    default:
      u8x8_SetGPIOResult(u8x8, 1);
      break;
  }
  return 1;
}

void u8g2_Init(u8g2_t *u8g2)
{
    // 向u8g2注册ssd1306驱动的iic处理函数
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay);
    
    u8g2_InitDisplay(u8g2);
    u8g2_SetPowerSave(u8g2, 0);
    u8g2_ClearBuffer(u8g2);
}
