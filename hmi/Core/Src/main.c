/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - XCalibur ARES (180 Deg Flipped Vertical)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define LCD_WIDTH   800
#define LCD_HEIGHT  480

/* --- Vertical UI Settings --- */
#define UI_WIDTH    480
#define UI_HEIGHT   800
#define BANNER_HEIGHT 80

/* Using the STM32H7's Internal AXI SRAM, bypassing the broken external chip! */
#define INTERNAL_FB_ADDRESS  0x24000000UL  

/* --- Colors (RGB565) --- */
#define COLOR_WHITE 0xFFFF
#define COLOR_BLUE  0x34BF 
#define COLOR_BLACK 0x0000

static uint16_t *fb = (uint16_t *)INTERNAL_FB_ADDRESS; 

/* Private variables ---------------------------------------------------------*/
LTDC_HandleTypeDef hltdc;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim4;

/* 8x8 Font Table for "XCALIBUR ARES " */
const uint8_t font8x8_ares[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Space
    0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81, // X
    0x3C, 0x42, 0x80, 0x80, 0x80, 0x80, 0x42, 0x3C, // C
    0x18, 0x24, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42, // A
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFE, // L
    0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, // I
    0xFC, 0x42, 0x42, 0x7C, 0x42, 0x42, 0x42, 0xFC, // B
    0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, // U
    0xFC, 0x42, 0x42, 0x7C, 0x48, 0x44, 0x42, 0x42, // R
    0xFE, 0x80, 0x80, 0xF8, 0x80, 0x80, 0x80, 0xFE, // E
    0x3E, 0x40, 0x40, 0x3C, 0x02, 0x02, 0x42, 0x3C  // S
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
void LCD_InitLayer(void);
void GUI_Framework(void);

void UI_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void UI_DrawString(uint16_t x, uint16_t y, const char* str, uint16_t color);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  
  // Ensure LEDs start OFF
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /* Note: External SDRAM, FMC, and MPU are completely bypassed and unused */
  
  /* Turn on Display Hardware */
  HAL_GPIO_WritePin(DISP_GPIO_Port, DISP_Pin, GPIO_PIN_SET);
  
  /* Initialize LTDC and Framebuffer */
  MX_LTDC_Init();
  LCD_InitLayer(); 
  
  /* Initialize Peripherals */
  MX_SPI1_Init();
  MX_TIM4_Init();

  /* Turn on Backlight (TIM4 CH1) */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 15000); // 50% Brightness

  /* Draw the UI Banner */
  GUI_Framework();

  /* Infinite loop */
  while (1)
  {
    // Heartbeat 
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    HAL_Delay(500); 
  }
}

/**
  * @brief Translates Vertical UI coordinates (Flipped 180 degrees)
  */
void UI_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    for (uint16_t i = 0; i < h; i++) {
        uint16_t log_y = y + i;
        if (log_y >= BANNER_HEIGHT) break; // Protect vertical banner bounds
        
        for (uint16_t j = 0; j < w; j++) {
            uint16_t log_x = x + j;
            if (log_x >= UI_WIDTH) break; // Protect horizontal width
            
            // MATH UPDATE: Rotate 90 degrees Counter-Clockwise (180 flip from before)
            uint16_t buf_x = log_y; 
            uint16_t buf_y = (LCD_HEIGHT - 1) - log_x;
            
            fb[buf_y * BANNER_HEIGHT + buf_x] = color;
        }
    }
}

/**
  * @brief Prints a string to the screen
  */
void UI_DrawString(uint16_t x, uint16_t y, const char* str, uint16_t color) 
{
    while (*str) {
        uint8_t glyph = 0;
        char c = *str;
        
        if (c == ' ') glyph = 0;
        else if (c == 'X') glyph = 1;
        else if (c == 'C') glyph = 2;
        else if (c == 'A') glyph = 3;
        else if (c == 'L') glyph = 4;
        else if (c == 'I') glyph = 5;
        else if (c == 'B') glyph = 6;
        else if (c == 'U') glyph = 7;
        else if (c == 'R') glyph = 8;
        else if (c == 'E') glyph = 9;
        else if (c == 'S') glyph = 10;

        for (int i = 0; i < 8; i++) {
            uint8_t row = font8x8_ares[glyph * 8 + i];
            for (int j = 0; j < 8; j++) {
                if (row & (0x80 >> j)) {
                    // Draw 3x3 pixel blocks to scale up the text
                    UI_FillRect(x + (j * 3), y + (i * 3), 3, 3, color);
                }
            }
        }
        x += 30; // Move right for next letter
        str++;
    }
}

/**
  * @brief Make general framework for gui
  */
void GUI_Framework(void)
{
  // 1. Fill the entire 80x480 internal buffer with Blue
  for (uint32_t i = 0; i < (UI_WIDTH * BANNER_HEIGHT); i++)
  {
      fb[i] = COLOR_BLUE;
  }

  // 2. Draw Title Text inside the portrait blue bar
  // Center roughly: X = 45, Y = 28
  UI_DrawString(45, 28, "XCALIBUR ARES", COLOR_WHITE);

  // 3. Push the pixels from the CPU Cache to our internal SRAM
  SCB_CleanDCache_by_Addr((uint32_t*)fb, (UI_WIDTH * BANNER_HEIGHT * 2));
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
      Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
      Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLL3.PLL3M = 4;        
  PeriphClkInitStruct.PLL3.PLL3N = 120;      
  PeriphClkInitStruct.PLL3.PLL3P = 2;        
  PeriphClkInitStruct.PLL3.PLL3Q = 2;        
  PeriphClkInitStruct.PLL3.PLL3R = 8;        
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
      Error_Handler();
  }
}

/**
  * @brief LTDC Initialization Function
  */
static void MX_LTDC_Init(void)
{
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 48;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 88;
  hltdc.Init.AccumulatedVBP = 32;
  hltdc.Init.AccumulatedActiveW = 888;
  hltdc.Init.AccumulatedActiveH = 512;
  hltdc.Init.TotalWidth = 928;
  hltdc.Init.TotalHeigh = 525;
  
  // ---------------------------------------------------------
  // Let the hardware paint the background White for us
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  // ---------------------------------------------------------
  
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Setup the LCD layer configuration
  */
void LCD_InitLayer(void)
{
    LTDC_LayerCfgTypeDef pLayerCfg = {0};

    // ---------------------------------------------------------
    // MATH UPDATE: Move our small 80-pixel layer to the physical LEFT edge of the LCD.
    // When you rotate the device 180 degrees from before (Ribbon at Top), 
    // this becomes your TOP banner!
    pLayerCfg.WindowX0      = 0; 
    pLayerCfg.WindowX1      = BANNER_HEIGHT - 1;         // 79
    pLayerCfg.WindowY0      = 0;                         
    pLayerCfg.WindowY1      = LCD_HEIGHT - 1;            // 479
    
    pLayerCfg.PixelFormat   = LTDC_PIXEL_FORMAT_RGB565; 
    pLayerCfg.FBStartAdress = INTERNAL_FB_ADDRESS; 
    
    pLayerCfg.ImageWidth    = BANNER_HEIGHT; 
    pLayerCfg.ImageHeight   = LCD_HEIGHT; 
    // ---------------------------------------------------------
    
    pLayerCfg.Alpha         = 255;          /* opaque */
    pLayerCfg.Backcolor.Blue  = 0;
    pLayerCfg.Backcolor.Green = 0;
    pLayerCfg.Backcolor.Red   = 0;

    if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief SPI1 Initialization Function
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  */
static void MX_TIM4_Init(void)
{
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim4);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RED_LED_Pin|GREEN_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(UP_BTTN_GPIO_Port, UP_BTTN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DISP_GPIO_Port, DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RED_LED_Pin GREEN_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SELECT_BTTN_Pin RIGHT_BTTN_Pin LEFT_BTTN_Pin DOWN_BTTN_Pin */
  GPIO_InitStruct.Pin = SELECT_BTTN_Pin|RIGHT_BTTN_Pin|LEFT_BTTN_Pin|DOWN_BTTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UP_BTTN_Pin */
  GPIO_InitStruct.Pin = UP_BTTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UP_BTTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DISP_Pin */
  GPIO_InitStruct.Pin = DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DISP_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
      HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
      HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
      
      for(volatile uint32_t i = 0; i < 20000000; i++) {} 
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */