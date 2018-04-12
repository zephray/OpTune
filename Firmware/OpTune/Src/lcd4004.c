#include "main.h"
#include "stm32f3xx_hal.h"
#include "lcd4004.h"

#define LCD_D4_OUT(x) { GPIOA->ODR &= 0xFEFF; GPIOA->ODR |= ((uint32_t)x)<<8;}
#define LCD_D5_OUT(x) { GPIOA->ODR &= 0xFDFF; GPIOA->ODR |= ((uint32_t)x)<<9;}
#define LCD_D6_OUT(x) { GPIOA->ODR &= 0xFBFF; GPIOA->ODR |= ((uint32_t)x)<<10;}
#define LCD_D7_OUT(x) { GPIOA->ODR &= 0xF7FF; GPIOA->ODR |= ((uint32_t)x)<<11;}
#define LCD_DATA_OUT(x) {GPIOA->ODR &= 0xF0FF; GPIOA->ODR |= ((uint32_t)x)<<8;}
#define LCD_RS_HIGH() GPIOB->BSRR = GPIO_PIN_13
#define LCD_RS_LOW()  GPIOB->BRR = GPIO_PIN_13
#define LCD_RW_HIGH() GPIOB->BSRR = GPIO_PIN_14
#define LCD_RW_LOW()  GPIOB->BRR = GPIO_PIN_14
#define LCD_E1_HIGH() GPIOB->BSRR = GPIO_PIN_15
#define LCD_E1_LOW()  GPIOB->BRR = GPIO_PIN_15
#define LCD_E2_HIGH() GPIOB->BSRR = GPIO_PIN_12
#define LCD_E2_LOW()  GPIOB->BRR = GPIO_PIN_12

#define WR_DELAY 200

void delay_cycles(size_t delay) {
  volatile size_t x = delay;
  while (x--);
}

void lcd_send_byte(unsigned char byte, LCD_TARGET target) {
  /*LCD_D7_OUT((byte >> 3)&0x01);
  LCD_D6_OUT((byte >> 2)&0x01);
  LCD_D5_OUT((byte >> 1)&0x01);
  LCD_D4_OUT((byte >> 0)&0x01);*/
  LCD_DATA_OUT(byte);
  LCD_RW_LOW();
  if ((target == LCD_CHIP1) || (target == LCD_BOTH))
    LCD_E1_HIGH();
  if ((target == LCD_CHIP2) || (target == LCD_BOTH))
    LCD_E2_HIGH();
  delay_cycles(WR_DELAY);
  if ((target == LCD_CHIP1) || (target == LCD_BOTH))
    LCD_E1_LOW();
  if ((target == LCD_CHIP2) || (target == LCD_BOTH))
    LCD_E2_LOW();
  delay_cycles(WR_DELAY);
}

void lcd_send_dat(unsigned char dat, LCD_TARGET target) {
  LCD_RS_HIGH();
  lcd_send_byte((dat >> 4)&0xF, target);
  lcd_send_byte(dat&0xF, target);
}

void lcd_send_cmd(unsigned char cmd, LCD_TARGET target) {
  LCD_RS_LOW();
  lcd_send_byte((cmd >> 4)&0xF, target);
  lcd_send_byte(cmd&0xF, target);
  HAL_Delay(2);
}

void lcd_init() {
  LCD_E1_LOW();
  LCD_E2_LOW();
  delay_cycles(100000);
  lcd_send_cmd(0x33, LCD_BOTH);
  lcd_send_cmd(0x32, LCD_BOTH);
  lcd_send_cmd(0x28, LCD_BOTH);
  lcd_send_cmd(0x28, LCD_BOTH);
  lcd_send_cmd(0x28, LCD_BOTH);
  lcd_send_cmd(0x0C, LCD_BOTH);
  lcd_send_cmd(0x01, LCD_BOTH);
  lcd_send_cmd(0x06, LCD_BOTH);
  lcd_send_cmd(0x80, LCD_BOTH);
}

void lcd_print(char *str) {
  while(*str){
    lcd_send_dat(*str++, LCD_CHIP1);
  }
}