
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "fatfs.h"
#include "lcd4004.h"
#include "mcp4728.h"
#include "mcp4012.h"

/* USER CODE BEGIN Includes */
#include "instruments.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS SD_FatFs;  /* File system object for SD card logical drive */
char SD_Path[4]; /* SD card logical drive path */

#define BUFFER_SIZE 512
#define SAMPLE_RATE 32000

static unsigned short audio_buffer_1[BUFFER_SIZE];
static unsigned short audio_buffer_2[BUFFER_SIZE];
unsigned short * audio_buffer_playback = audio_buffer_1;
unsigned short * audio_buffer_render = audio_buffer_2;
unsigned int playback_pointer = 0;
unsigned int render_pointer = 0;
volatile unsigned char buffer_full_flag = 0;

static unsigned char song_buffer[512]; // Hold 128 Instructions

#define CH_NUM 32
//#define LEVEL_MUL 2
#define LEVEL_DIV 2

typedef struct {
  unsigned char onoff; 
  unsigned short frequency; // Frequency in Hertz
  unsigned char volume; // Initial Volume, max = 100%, min = 0%
  unsigned char instrument;
  unsigned char envelope; // Calculated current volume
  unsigned long freq_counter; // Counter value
  unsigned long freq_reverse; // At which value it should reverse the output 
  unsigned long freq_reload;  // Initial value
  unsigned long envelope_counter; // Volume envelope counter
} sound_ch;

sound_ch ch[CH_NUM];

char message[30];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void delay_cycles(unsigned long delay) {
  volatile unsigned long x = delay;
  while (x--);
}

static int SDCard_Config(void)
{
  if(FATFS_LinkDriver(&USER_Driver, SD_Path) == 0)
  {
    /* Initialize the SD mounted on adafruit 1.8" TFT shield */
    if(BSP_SD_Init() != MSD_OK)
    {
      lcd_print("SD Init Failed. ");
      return -1;
    }  
    
    /* Check the mounted device */
    if(f_mount(&SD_FatFs, (TCHAR const*)"/", 0) != FR_OK)
    {
      lcd_print("FS Mount Failed. ");
      return -1;
    }  
    else
    {
      SD_CardInfo cardInfo;
      BSP_SD_GetCardInfo(&cardInfo);
      char buf[50];
      sprintf(buf, "SD Capacity: %d MB ", cardInfo.CardCapacity / 1024);
      lcd_print(buf);
      return 0;
      /* Initialize the Directory Files pointers (heap) */
    }
  }
  
  return -1;
}

void playFrame(unsigned char *buf) {
  lcd_send_cmd(0x8C, LCD_CHIP1); //Line 1
  for (int i = 0; i < 16; i++)
    lcd_send_dat(buf[i], LCD_CHIP1);
  lcd_send_cmd(0xCC, LCD_CHIP1); //Line 2
  for (int i = 16; i < 32; i++)
    lcd_send_dat(buf[i], LCD_CHIP1);
  lcd_send_cmd(0x40, LCD_CHIP1); //CGRAM
  for (int i = 32; i < 96; i++)
    lcd_send_dat(buf[i], LCD_CHIP1);
  lcd_send_cmd(0x8C, LCD_CHIP2); //Line 1
  for (int i = 96; i < 112; i++)
    lcd_send_dat(buf[i], LCD_CHIP2);
  lcd_send_cmd(0xCC, LCD_CHIP2); //Line 2
  for (int i = 112; i < 128; i++)
    lcd_send_dat(buf[i], LCD_CHIP2);
  lcd_send_cmd(0x40, LCD_CHIP2); //CGRAM
  for (int i = 128; i < 192; i++)
    lcd_send_dat(buf[i], LCD_CHIP2);
}

void playMovie() {
  FIL fil;        /* File object */
  FRESULT fr;     /* FatFs return code */
  static unsigned char buf[192*16]; // Hold 16 frame of image 
  
  fr = f_open(&fil, "out.bin", FA_READ);
  if (fr) {
    lcd_print("Unable to open file. ");
  }
    
  lcd_send_cmd(0x01, LCD_BOTH);
  HAL_Delay(5);
  
  unsigned int bytesRead = 1;
  int startTick = HAL_GetTick();
  int frameCount = 0;
  // Frame length = 42 42 41 42 42 41...
  while (bytesRead != 0) {
    f_read(&fil, (void *)buf, 192*16, &bytesRead);
    for (int i = 0; i < (bytesRead/192); i++) {
      playFrame(buf+i*192);
      frameCount ++;
      while (HAL_GetTick() < ((frameCount/3)*125 + (frameCount%3)*42)); //24FPS
    }
  }
}

void play_wave() {
  FIL fil;        /* File object */
  FRESULT fr;     /* FatFs return code */

  fr = f_open(&fil, "test.wav", FA_READ);
  if (fr) {
    lcd_print("Unable to open file. ");
  }
 
  unsigned int bytesRead = 1;
  int playCount = 0;
  while (bytesRead != 0) {
    f_read(&fil, (void *)audio_buffer_render, BUFFER_SIZE * 2, &bytesRead);
    for (int i = 0; i < BUFFER_SIZE; i++) audio_buffer_render[i] = audio_buffer_render[i] ^ 0x8000;
    buffer_full_flag = 1;
    playCount ++;
    lcd_set_xy(0,1);
    sprintf(message, "%d", playCount);
    lcd_print(message);
    while (buffer_full_flag);
    unsigned short *buffer_temp;
    buffer_temp = audio_buffer_playback;
    audio_buffer_playback = audio_buffer_render;
    audio_buffer_render = buffer_temp;
    playback_pointer = 0;
  }
}


void sound_set_pitch(int channel, unsigned long pitch) {
  ch[channel].frequency = pitch;
  ch[channel].freq_reload = SAMPLE_RATE / pitch;
  ch[channel].freq_counter = ch[channel].freq_reload;
  ch[channel].freq_reverse = ch[channel].freq_reload / 2;
  ch[channel].freq_reverse = ch[channel].freq_reload * instruments[ch[channel].instrument].duty / 256;
  ch[channel].envelope_counter = 0;
  lcd_set_xy(0, 1);
  sprintf(message, "P %d, %d", channel, pitch);
  lcd_print("            ");
  lcd_set_xy(0, 1);
  lcd_print(message);
}

void sound_set_instrument(int channel, unsigned char instrument) {
  ch[channel].envelope_counter = 0;
  ch[channel].instrument = instrument;
  lcd_set_xy(0, 1);
  sprintf(message, "I %d, %d", channel, instrument);
  lcd_print("            ");
  lcd_set_xy(0, 1);
  lcd_print(message);
}

void sound_set_volume(int channel, unsigned char volume) {
  /*ch[channel].volume = volume;*/
}

void sound_set_onoff(int channel, unsigned char onoff) {
  ch[channel].onoff = onoff;
}

void render_song() {
  unsigned long cs; //current sample
  
  // process envelope
  unsigned char a_length;
  unsigned char a_step;
  unsigned char d_length;
  unsigned char d_step;
  unsigned char s_length;
  unsigned char r_length;
  unsigned char r_step;
  
  for (int c = 0; c < CH_NUM; c++) {
    a_length = instruments[ch[c].instrument].a_length;
    a_step = ch[c].volume / a_step;
    d_length = instruments[ch[c].instrument].d_length;
    d_step = instruments[ch[c].instrument].d_step;
    s_length = instruments[ch[c].instrument].s_length;
    r_length = instruments[ch[c].instrument].r_length;
    r_step = (ch[c].volume - d_length * d_step) / r_length;
    
    if ((ch[c].envelope_counter) < (a_length - 1)) {
      ch[c].envelope += a_step;
    }
    else if ((ch[c].envelope_counter) == (a_length - 1)) {
      ch[c].envelope = ch[c].volume;
    }
    else if ((ch[c].envelope_counter) < (a_length + d_length)) {
      ch[c].envelope -= d_step;
    }
    else if ((ch[c].envelope_counter) < (a_length + d_length + s_length + r_length - 1)) {
      ch[c].envelope -= r_step;
    }
    else if ((ch[c].envelope_counter) == (a_length + d_length + s_length + r_length - 1)) {
      ch[c].envelope = 0;
    }
    
    ch[c].envelope_counter ++;
  }
  
  for (int s = 0; s < BUFFER_SIZE; s ++) {
    cs = 0;
    for (int c = 0; c < CH_NUM; c++) {
      if (ch[c].onoff == 0) continue;
      if (ch[c].freq_counter == 0) {
        ch[c].freq_counter = ch[c].freq_reload;
      }
      else if (ch[c].freq_counter >= ch[c].freq_reverse) {
        cs += ch[c].envelope;
        //cs += 255;
      }
      else {
        // do nothing
      }
      ch[c].freq_counter --;
    }
#ifdef LEVEL_MUL
    cs = cs * LEVEL_MUL;
#else
    cs = cs / LEVEL_DIV;
#endif
    
    audio_buffer_render[s] = cs;
  }
}

// SNG Format
// Each operation is four bytes, one byte instruction, one byte channel, 2 bytes parameters
// Operation List
// 0x00 NOP
// 0x01 DELAY MS
// 0x02 SET PAN
// 0x03 SET VOL 
// 0x04 SET PIT
// 0x05 SET INSTRUMENT
// 0x06 SET CH ON/OFF 0x0001 on/ 0x0000 off
// 0x07 MOD INSTRUMENT DUTY RESERVED
// 0x08 MOD INSTRUMENT ATTACK DECAY
// 0x09 MOD INSTRUMENT SUSTAIN RELEASE

void play_song() {
  FIL fil;        /* File object */
  FRESULT fr;     /* FatFs return code */

  fr = f_open(&fil, "test.sng", FA_READ);
  if (fr) {
    lcd_print("Unable to open file. ");
  }
 
  unsigned int bytesRead = 1;
  unsigned long delayStart;
  unsigned short *buffer_temp;

  for (int i = 0; i < CH_NUM; i++) {
    sound_set_instrument(i, 0);
    sound_set_onoff(i, 0);
    sound_set_volume(i, 255);
  }
  
  /*sound_set_pitch(0, 1000);
  sound_set_onoff(0, 1);
  
  sound_set_pitch(1, 2000);
  sound_set_onoff(1, 1);*/
  
  buffer_full_flag = 0;
  int playCount = 0;
  
  /*while (1) {
    render_song();
    buffer_full_flag = 1;
    while (buffer_full_flag);
    unsigned short *buffer_temp;
    buffer_temp = audio_buffer_playback;
    audio_buffer_playback = audio_buffer_render;
    audio_buffer_render = buffer_temp;
    playback_pointer = 0;
  }*/
  
  while (bytesRead != 0) {
    f_read(&fil, (void *)song_buffer, 512, &bytesRead);
    for (int i = 0; i < (bytesRead / 4); i++) {
      unsigned char op = song_buffer[i * 4 + 0];
      unsigned char c =  song_buffer[i * 4 + 1];
      unsigned short par = (((unsigned short)song_buffer[i * 4 + 2]) << 8) | 
        ((unsigned short)song_buffer[i * 4 + 3]);
      if (op != 0x01) {
        switch (op) {
          //case 0x03: sound_set_volume(c, par); break;
          case 0x04: sound_set_pitch(c, par); sound_set_onoff(c, 1); break;
          case 0x05: sound_set_instrument(c, par); break;
          case 0x06: sound_set_onoff(c, par); break;
        }
      }
      else {
        delayStart = HAL_GetTick();
        //lcd_set_xy(0, 1);
        //sprintf(message, "D %d", par);
        //lcd_print("            ");
        //lcd_set_xy(0, 1);
        //lcd_print(message);
        while ((HAL_GetTick() - delayStart) < par) {
          if (buffer_full_flag != 1) {
            buffer_temp = audio_buffer_playback;
            audio_buffer_playback = audio_buffer_render;
            audio_buffer_render = buffer_temp;
            playback_pointer = 0;
            render_song();
            buffer_full_flag = 1;
          }
        }
      }
    }
  }
}

void init_timer4() {
  __HAL_RCC_TIM4_CLK_ENABLE();
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim4);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQHandler()
{
    HAL_TIM_IRQHandler(&htim4);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, audio_buffer_playback[playback_pointer]);
    if (playback_pointer < BUFFER_SIZE - 1) {
      playback_pointer ++;
    } else {
      playback_pointer = 0;
      buffer_full_flag = 0;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */

int main(void)
{
  /* USER CODE BEGIN 1 */
  int sd_ready;
  /* USER CODE END 1 */
  
  /* MCU Configuration----------------------------------------------------------*/

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
  //MX_SPI1_Init();
  MX_I2C1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  //MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  dac_init(&hi2c1);
  dpot_init();
  
  sd_ready = SDCard_Config();
  
  if (sd_ready != 0) {
    while(1);
  }
  
  dpot_set(1, 20);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  init_timer4();
  //play_wave();
  play_song();
  HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
  
  //playMovie();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Noise wave generation on DAC OUT1 
    */
  /*if (HAL_DACEx_NoiseWaveGenerate(&hdac, DAC_CHANNEL_1, DAC_LFSRUNMASK_BIT0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }*/

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA7 PA8 PA9 PA10 
                           PA11 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB12 PB13 
                           PB14 PB15 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
