#ifndef __LCD4004_H__
#define __LCD4004_H__

typedef enum
{
  LCD_CMD = 0U,
  LCD_DAT
}LCD_DATA_TYPE;

typedef enum
{
  LCD_CHIP1 = 0,
  LCD_CHIP2,
  LCD_BOTH
}LCD_TARGET;

void lcd_init();
void lcd_send_dat(unsigned char dat, LCD_TARGET target);
void lcd_send_cmd(unsigned char cmd, LCD_TARGET target);
void lcd_set_xy(int x, int y);
void lcd_print(char *str);

#endif