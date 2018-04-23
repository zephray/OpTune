#include "main.h"
#include "stm32f3xx_hal.h"
#include "mcp4012.h"

uint8_t pot_values[6];

void dpot_select_all() {
  CS1_LOW();
  CS2_LOW();
  CS3_LOW();
  CS4_LOW();
  CS5_LOW();
  CS6_LOW();
}

void dpot_deselect_all() {
  CS1_HIGH();
  CS2_HIGH();
  CS3_HIGH();
  CS4_HIGH();
  CS5_HIGH();
  CS6_HIGH();
}

void dpot_select(int channel) {
  if (channel == 0) {CS1_LOW();} else
  if (channel == 1) {CS2_LOW();} else
  if (channel == 2) {CS3_LOW();} else
  if (channel == 3) {CS4_LOW();} else
  if (channel == 4) {CS5_LOW();} else
  if (channel == 5) {CS6_LOW();}
}

void dpot_deselect(int channel) {
  if (channel == 0) {CS1_HIGH();} else
  if (channel == 1) {CS2_HIGH();} else
  if (channel == 2) {CS3_HIGH();} else
  if (channel == 3) {CS4_HIGH();} else
  if (channel == 4) {CS5_HIGH();} else
  if (channel == 5) {CS6_HIGH();}
}

void dpot_init() {
  UD_HIGH();
  dpot_select_all();
  UD_LOW();
  for (int i = 0; i < 64 - 1; i++) {
    UD_HIGH();
    UD_LOW();
  }
  UD_HIGH();
  dpot_deselect_all();
  UD_LOW();
  for (int i = 0; i < 6; i++)
     pot_values[i] = 63;
}

void dpot_set(int channel, unsigned char value) {
  if (value > 63) value = 63;
  
  if (pot_values[channel] < value) {
    UD_HIGH();
    dpot_select(channel);
    UD_LOW();
    for (int i = 0; i < (value - pot_values[channel] - 1); i ++) {
      UD_HIGH();
      UD_LOW();
    }
    UD_HIGH();
    dpot_deselect(channel);
    UD_LOW();
  }  
  else if (pot_values[channel] > value) {
    UD_LOW();
    dpot_select(channel);
    UD_HIGH();
    for (int i = 0; i < (pot_values[channel] - value - 1); i ++) {
      UD_LOW();
      UD_HIGH();
    }
    UD_LOW();
    dpot_deselect(channel);
    UD_HIGH();
  }
}