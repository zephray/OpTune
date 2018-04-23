#ifndef __MCP4012_H__
#define __MCP4012_H__

#define UD_HIGH()  GPIOB->BSRR = GPIO_PIN_1; delay_cycles(5)
#define UD_LOW()   GPIOB->BRR  = GPIO_PIN_1; delay_cycles(5)
#define CS1_HIGH() GPIOB->BSRR = GPIO_PIN_0; delay_cycles(5)
#define CS1_LOW()  GPIOB->BRR  = GPIO_PIN_0; delay_cycles(5)
#define CS2_HIGH() GPIOA->BSRR = GPIO_PIN_7; delay_cycles(5)
#define CS2_LOW()  GPIOA->BRR  = GPIO_PIN_7; delay_cycles(5)
#define CS3_HIGH() GPIOA->BSRR = GPIO_PIN_2; delay_cycles(5)
#define CS3_LOW()  GPIOA->BRR  = GPIO_PIN_2; delay_cycles(5)
#define CS4_HIGH() GPIOA->BSRR = GPIO_PIN_3; delay_cycles(5)
#define CS4_LOW()  GPIOA->BRR  = GPIO_PIN_3; delay_cycles(5)
#define CS5_HIGH() GPIOA->BSRR = GPIO_PIN_0; delay_cycles(5)
#define CS5_LOW()  GPIOA->BRR  = GPIO_PIN_0; delay_cycles(5)
#define CS6_HIGH() GPIOA->BSRR = GPIO_PIN_1; delay_cycles(5)
#define CS6_LOW()  GPIOA->BRR  = GPIO_PIN_1; delay_cycles(5)

void dpot_init();
void dpot_set(int channel, unsigned char value);

#endif