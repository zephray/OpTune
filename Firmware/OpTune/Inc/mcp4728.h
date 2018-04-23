#ifndef __MCP4728_H__
#define __MCP4728_H__

#define LDAC_HIGH() GPIOB->BSRR = GPIO_PIN_9
#define LDAC_LOW()  GPIOB->BRR = GPIO_PIN_9

int dac_init(I2C_HandleTypeDef *h);

#endif