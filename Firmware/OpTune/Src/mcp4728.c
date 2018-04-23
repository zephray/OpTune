#include "main.h"
#include "stm32f3xx_hal.h"
#include "mcp4728.h"

I2C_HandleTypeDef *hi2c;
uint8_t dac_channels[8] = {0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00};

int dac_init(I2C_HandleTypeDef *h) {
  hi2c = h;

  LDAC_HIGH();
  
  HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(h,0x60<<1,dac_channels,8,100); 
  
  HAL_Delay(1);
  LDAC_LOW();
  HAL_Delay(1);
  LDAC_HIGH();
  
  if (result == HAL_ERROR) return -1;
  else if (result == HAL_TIMEOUT) return -2;
  else return 0;
}