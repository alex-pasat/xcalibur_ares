/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - XCalibur ARES (Dynamic UI, Musical PWM)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"
#include <stdint.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
// Physical screen dimensions
#define LCD_WIDTH   800
#define LCD_HEIGHT  480

// Knife Image Dimensions
#define IMAGE_WIDTH  200
#define IMAGE_HEIGHT 200

// Splash Logo Dimensions
#define SPLASH_WIDTH  400
#define SPLASH_HEIGHT 427

// Link to your image data in the other files. 
extern const uint8_t chef_knife_image[]; 
extern const uint8_t paring_knife_image[]; 
extern const uint8_t gyuto_knife_image[]; 
extern const uint8_t utility_knife_image[]; 
extern const uint8_t xcalibur_image[]; 
extern const uint8_t sharpening_frame0[];
extern const uint8_t sharpening_frame1[];
extern const uint8_t sharpening_frame2[];
extern const uint8_t sharpening_frame3[];
extern const uint8_t sharpening_frame4[];
extern const uint8_t sharpening_frame5[];
extern const uint8_t hands_off_image[];
extern const uint8_t knife_insert_image[];
extern const uint8_t xcalibur_green_image[];

const uint8_t *sharpening_frames[] = {
    sharpening_frame0,
    sharpening_frame1,
    sharpening_frame2,
    sharpening_frame3,
    sharpening_frame4,
    sharpening_frame5
};

/* --- Vertical UI Settings --- */
#define UI_WIDTH    480
#define UI_HEIGHT   800
#define BANNER_HEIGHT 80

/* SPI Commands*/
#define SPI_CMD_SEND_KNIFE    0x10  
#define SPI_CMD_POLL_STATUS   0x20  

// --- SPI PROTOCOL COMMANDS FROM MAIN BOARD ---
typedef enum {
  ROBOT_HMI_CMD_NONE               = 0x00,
  ROBOT_HMI_CMD_KNIFE_CLAMPED      = 0x01,
  ROBOT_HMI_CMD_KNIFE_DONE         = 0x02,
  ROBOT_HMI_CMD_KNIFE_REMOVED      = 0x03,
  ROBOT_HMI_CMD_KNIFE_NOT_DETECTED = 0x04,
  USB_CONNECTED                    = 0x05
} robot_hmi_command_t;

/* Using the STM32H7's Internal AXI SRAM */
#define INTERNAL_FB_ADDRESS  0x24000000UL  

// FRAMEBUFFER POINTER - Layer 0 uses L8 Mode (1 byte per pixel)
static uint8_t *fb = (uint8_t *)INTERNAL_FB_ADDRESS; 

/* --- L8 Color Palette Indices (Layer 0) --- */
#define L8_BLACK 0
#define L8_BLUE  1
#define L8_WHITE 2
#define L8_RED   3

/* --- State Machine Defintions --- */
// Define encompassing states for the UI
typedef enum {
    STATE_KNIFE_SELECTION,
    STATE_INSERTION,
    STATE_TOOL_LET_GO,
    STATE_SHARPENING,
    STATE_DONE_REMOVE
} UI_State_t;

// Knife Selection
typedef enum {
    NONE,
    CHEF_KNIFE,
    PARING_KNIFE,
    GYUTO_KNIFE,
    JAPANESE_UTILITY_KNIFE,
} KnifeType_t;

// Initialize our starting states
UI_State_t current_ui_state = STATE_KNIFE_SELECTION;
KnifeType_t current_knife = CHEF_KNIFE;

/* Private variables ---------------------------------------------------------*/
LTDC_HandleTypeDef hltdc;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim4;

/* Complete 8x8 ASCII Font Table (Characters 32 to 126) */
const uint8_t font8x8_ascii[96][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 32 ' '
    {0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00}, // 33 '!'
    {0x66, 0x66, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00}, // 34 '"'
    {0x6C, 0x6C, 0xFE, 0x6C, 0xFE, 0x6C, 0x6C, 0x00}, // 35 '#'
    {0x18, 0x3E, 0x60, 0x3C, 0x06, 0x7C, 0x18, 0x00}, // 36 '$'
    {0x00, 0xC6, 0xCC, 0x18, 0x30, 0x66, 0xC6, 0x00}, // 37 '%'
    {0x38, 0x6C, 0x38, 0x76, 0xDC, 0xCC, 0x76, 0x00}, // 38 '&'
    {0x18, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00}, // 39 '''
    {0x0C, 0x18, 0x30, 0x30, 0x30, 0x18, 0x0C, 0x00}, // 40 '('
    {0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x18, 0x30, 0x00}, // 41 ')'
    {0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00}, // 42 '*'
    {0x00, 0x18, 0x18, 0x7E, 0x18, 0x18, 0x00, 0x00}, // 43 '+'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30}, // 44 ','
    {0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00}, // 45 '-'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00}, // 46 '.'
    {0x00, 0x02, 0x06, 0x1C, 0x38, 0x60, 0x40, 0x00}, // 47 '/'
    {0x3C, 0x66, 0x6E, 0x76, 0x66, 0x66, 0x3C, 0x00}, // 48 '0'
    {0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00}, // 49 '1'
    {0x3C, 0x66, 0x06, 0x1C, 0x30, 0x60, 0x7E, 0x00}, // 50 '2'
    {0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x3C, 0x00}, // 51 '3'
    {0x1C, 0x3C, 0x6C, 0xCC, 0xFE, 0x0C, 0x0C, 0x00}, // 52 '4'
    {0x7E, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C, 0x00}, // 53 '5'
    {0x3C, 0x66, 0x60, 0x7C, 0x66, 0x66, 0x3C, 0x00}, // 54 '6'
    {0x7E, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x00}, // 55 '7'
    {0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x3C, 0x00}, // 56 '8'
    {0x3C, 0x66, 0x66, 0x3E, 0x06, 0x66, 0x3C, 0x00}, // 57 '9'
    {0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x00}, // 58 ':'
    {0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x30}, // 59 ';'
    {0x0C, 0x18, 0x30, 0x60, 0x30, 0x18, 0x0C, 0x00}, // 60 '<'
    {0x00, 0x00, 0x7E, 0x00, 0x7E, 0x00, 0x00, 0x00}, // 61 '='
    {0x30, 0x18, 0x0C, 0x06, 0x0C, 0x18, 0x30, 0x00}, // 62 '>'
    {0x3C, 0x66, 0x06, 0x1C, 0x18, 0x00, 0x18, 0x00}, // 63 '?'
    {0x3C, 0x66, 0x6E, 0x6E, 0x60, 0x66, 0x3C, 0x00}, // 64 '@'
    {0x18, 0x3C, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x00}, // 65 'A'
    {0x7C, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x7C, 0x00}, // 66 'B'
    {0x3C, 0x66, 0x60, 0x60, 0x60, 0x66, 0x3C, 0x00}, // 67 'C'
    {0x78, 0x6C, 0x66, 0x66, 0x66, 0x6C, 0x78, 0x00}, // 68 'D'
    {0x7E, 0x60, 0x60, 0x78, 0x60, 0x60, 0x7E, 0x00}, // 69 'E'
    {0x7E, 0x60, 0x60, 0x78, 0x60, 0x60, 0x60, 0x00}, // 70 'F'
    {0x3C, 0x66, 0x60, 0x6E, 0x66, 0x66, 0x3C, 0x00}, // 71 'G'
    {0x66, 0x66, 0x66, 0x7E, 0x66, 0x66, 0x66, 0x00}, // 72 'H'
    {0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00}, // 73 'I'
    {0x06, 0x06, 0x06, 0x06, 0x06, 0x66, 0x3C, 0x00}, // 74 'J'
    {0x66, 0x6C, 0x78, 0x70, 0x78, 0x6C, 0x66, 0x00}, // 75 'K'
    {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E, 0x00}, // 76 'L'
    {0x63, 0x77, 0x7F, 0x6B, 0x63, 0x63, 0x63, 0x00}, // 77 'M'
    {0x66, 0x76, 0x7E, 0x7E, 0x6E, 0x66, 0x66, 0x00}, // 78 'N'
    {0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00}, // 79 'O'
    {0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x00}, // 80 'P'
    {0x3C, 0x66, 0x66, 0x66, 0x66, 0x6E, 0x3C, 0x02}, // 81 'Q'
    {0x7C, 0x66, 0x66, 0x7C, 0x6C, 0x66, 0x66, 0x00}, // 82 'R'
    {0x3C, 0x66, 0x60, 0x3C, 0x06, 0x66, 0x3C, 0x00}, // 83 'S'
    {0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00}, // 84 'T'
    {0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00}, // 85 'U'
    {0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00}, // 86 'V'
    {0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00}, // 87 'W'
    {0x66, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x66, 0x00}, // 88 'X'
    {0x66, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x00}, // 89 'Y'
    {0x7E, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x7E, 0x00}, // 90 'Z'
    {0x3C, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3C, 0x00}, // 91 '['
    {0x00, 0x40, 0x60, 0x38, 0x1C, 0x06, 0x02, 0x00}, // 92 '\'
    {0x3C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x3C, 0x00}, // 93 ']'
    {0x18, 0x3C, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00}, // 94 '^'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF}, // 95 '_'
    {0x30, 0x18, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00}, // 96 '`'
    {0x00, 0x00, 0x3C, 0x06, 0x3E, 0x66, 0x3E, 0x00}, // 97 'a'
    {0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x7C, 0x00}, // 98 'b'
    {0x00, 0x00, 0x3C, 0x60, 0x60, 0x60, 0x3C, 0x00}, // 99 'c'
    {0x06, 0x06, 0x3E, 0x66, 0x66, 0x66, 0x3E, 0x00}, // 100 'd'
    {0x00, 0x00, 0x3C, 0x66, 0x7E, 0x60, 0x3C, 0x00}, // 101 'e'
    {0x1C, 0x30, 0x7C, 0x30, 0x30, 0x30, 0x30, 0x00}, // 102 'f'
    {0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x3C}, // 103 'g'
    {0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x00}, // 104 'h'
    {0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x3C, 0x00}, // 105 'i'
    {0x06, 0x00, 0x06, 0x06, 0x06, 0x66, 0x3C, 0x00}, // 106 'j'
    {0x60, 0x60, 0x66, 0x6C, 0x78, 0x6C, 0x66, 0x00}, // 107 'k'
    {0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00}, // 108 'l'
    {0x00, 0x00, 0x76, 0x7F, 0x6B, 0x6B, 0x6B, 0x00}, // 109 'm'
    {0x00, 0x00, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x00}, // 110 'n'
    {0x00, 0x00, 0x3C, 0x66, 0x66, 0x66, 0x3C, 0x00}, // 111 'o'
    {0x00, 0x00, 0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60}, // 112 'p'
    {0x00, 0x00, 0x3E, 0x66, 0x66, 0x3E, 0x06, 0x06}, // 113 'q'
    {0x00, 0x00, 0x7C, 0x66, 0x60, 0x60, 0x60, 0x00}, // 114 'r'
    {0x00, 0x00, 0x3C, 0x60, 0x3C, 0x06, 0x3C, 0x00}, // 115 's'
    {0x30, 0x30, 0x7C, 0x30, 0x30, 0x30, 0x1C, 0x00}, // 116 't'
    {0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x3E, 0x00}, // 117 'u'
    {0x00, 0x00, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00}, // 118 'v'
    {0x00, 0x00, 0x63, 0x6B, 0x7F, 0x3E, 0x36, 0x00}, // 119 'w'
    {0x00, 0x00, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0x00}, // 120 'x'
    {0x00, 0x00, 0x66, 0x66, 0x66, 0x3E, 0x0C, 0x38}, // 121 'y'
    {0x00, 0x00, 0x7E, 0x0C, 0x18, 0x30, 0x7E, 0x00}, // 122 'z'
    {0x0E, 0x18, 0x18, 0x70, 0x18, 0x18, 0x0E, 0x00}, // 123 '{'
    {0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x00}, // 124 '|'
    {0x70, 0x18, 0x18, 0x0E, 0x18, 0x18, 0x70, 0x00}, // 125 '}'
    {0x3A, 0x6E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  // 126 '~'
};

/* Private function prototypes -----------------------------------------------*/
void MPU_Config(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
void LCD_InitLayer(void);

// UI & Audio Functions
void GUI_Framework(void);
void Update_Tool_UI(uint8_t show_ui, const char* title, const uint8_t* image_array, uint8_t text_size);
void L8_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t colorIndex);
void L8_DrawString(uint16_t x, uint16_t y, const char* str, uint8_t colorIndex, uint8_t scale);
void L8_DrawPixel(uint16_t x, uint16_t y, uint8_t colorIndex);
void Startup_Splash_Screen(void);
void Play_Startup_Tune(void);
uint8_t Button_Pressed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void SPI_Send_Tool_Selection(KnifeType_t knife);
robot_hmi_command_t SPI_Poll_Main_Board(void);
void L8_DrawProgressBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t percent);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* 1. Configure MPU and Reset Peripherals */
  MPU_Config();
  HAL_Init();

  /* 2. Configure the system clock */
  SystemClock_Config();

  /* 3. Initialize Basic Peripherals */
  MX_GPIO_Init();
  
  // Keep LEDs OFF initially
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /* Turn on Display Hardware */
  HAL_GPIO_WritePin(DISP_GPIO_Port, DISP_Pin, GPIO_PIN_SET);
  
  /* Initialize LTDC and Framebuffer */
  MX_LTDC_Init();
  LCD_InitLayer(); 
  
  /* Initialize Comms and PWM */
  MX_SPI1_Init();
  MX_TIM4_Init(); 

  /* --- INITIALIZE UI --- */
  uint32_t sharpening_start_time = 0;
  
  /* Show Splash Logo and Play Audio Chime */
  Startup_Splash_Screen();
  
  /* 1. Draw the static UI Framework (Blue Banner) */
  GUI_Framework();

  /* 2. Draw Dynamic UI (Arrows, Text) and link Image to Layer 1 */
  Update_Tool_UI(1, "German Steel Chef Knife", chef_knife_image, 2);

  /* 3. Force Hardware Update to sync both layers */
  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);

  /* Infinite loop */
  while (1)
  {   
      // =========================================================
      // MAIN UI STATE MACHINE
      // =========================================================
      
      switch (current_ui_state) 
      {
          case STATE_KNIFE_SELECTION:
              HAL_GPIO_WritePin(GPIOE, RED_LED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOE, GREEN_LED_Pin, GPIO_PIN_SET);
              // ---------------------------------------------------
              // SUB-FSM: KNIFE SCROLLING
              // ---------------------------------------------------
              if (Button_Pressed(RIGHT_BTTN_GPIO_Port, RIGHT_BTTN_Pin)) {
                  if (current_knife == CHEF_KNIFE) { current_knife = PARING_KNIFE; Update_Tool_UI(1, "Paring Knife", paring_knife_image, 2); }
                  else if (current_knife == PARING_KNIFE) { current_knife = GYUTO_KNIFE; Update_Tool_UI(1, "Japanese Gyuto Knife", gyuto_knife_image, 2); }
                  else if (current_knife == GYUTO_KNIFE) { current_knife = JAPANESE_UTILITY_KNIFE; Update_Tool_UI(1, "Japanese Utility Knife", utility_knife_image, 2); }
                  else if (current_knife == JAPANESE_UTILITY_KNIFE) { current_knife = CHEF_KNIFE; Update_Tool_UI(1, "German Chef Knife", chef_knife_image, 2); }
                  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
              }
              else if (Button_Pressed(LEFT_BTTN_GPIO_Port, LEFT_BTTN_Pin)) {
                  if (current_knife == CHEF_KNIFE) { current_knife = JAPANESE_UTILITY_KNIFE; Update_Tool_UI(1, "Japanese Utility", utility_knife_image, 2); }
                  else if (current_knife == JAPANESE_UTILITY_KNIFE) { current_knife = GYUTO_KNIFE; Update_Tool_UI(1, "Gyuto Knife", gyuto_knife_image, 2); }
                  else if (current_knife == GYUTO_KNIFE) { current_knife = PARING_KNIFE; Update_Tool_UI(1, "Paring Knife", paring_knife_image, 2); }
                  else if (current_knife == PARING_KNIFE) { current_knife = CHEF_KNIFE; Update_Tool_UI(1, "German Chef Knife", chef_knife_image, 2); }
                  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
              }
              // ---------------------------------------------------
              // SELECTION TRIGGER -> Transition to next System State
              // ---------------------------------------------------
              else if (Button_Pressed(SELECT_BTTN_GPIO_Port, SELECT_BTTN_Pin)) {
                  SPI_Send_Tool_Selection(current_knife);
                  Update_Tool_UI(0, "Please Enter Knife", knife_insert_image, 2); 
                  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
                  HAL_Delay(500);
                  current_ui_state = STATE_INSERTION;
              }
              break;

          case STATE_INSERTION:
              // Blink the green LED to show it's actively waiting
              HAL_GPIO_WritePin(GPIOE, GREEN_LED_Pin, GPIO_PIN_SET);
              HAL_Delay (200);
              HAL_GPIO_WritePin(GPIOE, GREEN_LED_Pin, GPIO_PIN_RESET);
              HAL_Delay (200);

              // 1. Poll the main board exactly ONCE and save the answer!
              robot_hmi_command_t status = SPI_Poll_Main_Board();

              // 2. Check the saved answer
              if (status == ROBOT_HMI_CMD_KNIFE_NOT_DETECTED) {
                  
                  // Show the timeout error message
                  Update_Tool_UI(0, "TIMEOUT - NO KNIFE!", knife_insert_image, 2); 
                  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
                  HAL_Delay(2000); // Give the user 2 seconds to read the warning
                  
                  // 3. THE FIX: Manually redraw the main menu BEFORE changing states!
                  switch(current_knife) {
                      case CHEF_KNIFE: Update_Tool_UI(1, "German Chef Knife", chef_knife_image,2); break;
                      case PARING_KNIFE: Update_Tool_UI(1, "Paring Knife", paring_knife_image,2); break;
                      case GYUTO_KNIFE: Update_Tool_UI(1, "Gyuto Knife", gyuto_knife_image,2); break;
                      case JAPANESE_UTILITY_KNIFE: Update_Tool_UI(1, "Japanese Utility", utility_knife_image,2); break;
                  }
                  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
                  
                  // Now it is safe to return to the selection loop
                  current_ui_state = STATE_KNIFE_SELECTION;
              }
              else if (status == ROBOT_HMI_CMD_KNIFE_CLAMPED ){
                  
                  Update_Tool_UI(0, "Let go of knife!", hands_off_image, 2);
                  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
                  current_ui_state = STATE_TOOL_LET_GO;
                  
              }
              break;

          case STATE_TOOL_LET_GO:
              for (int i = 0; i < 10; i++) {
                HAL_GPIO_WritePin(GPIOE, RED_LED_Pin, GPIO_PIN_SET);
                HAL_Delay (200);
                HAL_GPIO_WritePin(GPIOE, RED_LED_Pin, GPIO_PIN_RESET);
                HAL_Delay (200);
              }
              for (int i = 0; i <= 5; i++) {
                  Update_Tool_UI(0, "Sharpening in progress...", sharpening_frames[i], 2);
                  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
                  HAL_Delay(100);
              }
              sharpening_start_time = HAL_GetTick();
              current_ui_state = STATE_SHARPENING;
              break;

          case STATE_SHARPENING:
              HAL_GPIO_WritePin(GPIOE, RED_LED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOE, GREEN_LED_Pin, GPIO_PIN_RESET);

              uint32_t current_time = HAL_GetTick();
              uint32_t elapsed_ms = current_time - sharpening_start_time;
              uint8_t progress_percent = (elapsed_ms / 20000) * 10; 
              if (progress_percent > 90) { progress_percent = 90; }

              for (int i = 0; i <= 5; i++) {
                  Update_Tool_UI(0, "Sharpening in progress...", sharpening_frames[i], 2);
                  L8_DrawProgressBar(90, 580, 300, 30, progress_percent);
                  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
                  HAL_Delay(100);
              }
              if (SPI_Poll_Main_Board() == ROBOT_HMI_CMD_KNIFE_DONE) { 
                  L8_DrawProgressBar(90, 580, 300, 30, 100);
                  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
                  HAL_Delay(600);
                  Play_Startup_Tune(); 
                  Update_Tool_UI(0, "Knife sharpened! Remove knife.", xcalibur_green_image, 2);
                  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
                  current_ui_state = STATE_DONE_REMOVE;
              }
              break;

          case STATE_DONE_REMOVE:
                HAL_GPIO_WritePin(GPIOE, GREEN_LED_Pin, GPIO_PIN_SET);
                HAL_Delay (200);
                HAL_GPIO_WritePin(GPIOE, GREEN_LED_Pin, GPIO_PIN_RESET);
                HAL_Delay (200);
              if (SPI_Poll_Main_Board() == ROBOT_HMI_CMD_KNIFE_REMOVED) { 
                  sharpening_start_time = 0;
                  switch(current_knife) {
                      case CHEF_KNIFE: Update_Tool_UI(1, "German Chef Knife", chef_knife_image,2); break;
                      case PARING_KNIFE: Update_Tool_UI(1, "Paring Knife", paring_knife_image,2); break;
                      case GYUTO_KNIFE: Update_Tool_UI(1, "Gyuto Knife", gyuto_knife_image,2); break;
                      case JAPANESE_UTILITY_KNIFE: Update_Tool_UI(1, "Japanese Utility", utility_knife_image,2); break;
                  }
                  HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
                  current_ui_state = STATE_KNIFE_SELECTION;
              }
              break;
      }
  }
}

void SPI_Send_Tool_Selection(KnifeType_t knife){
  uint8_t tx_buffer[1];
  tx_buffer[0] = (uint8_t)knife; // Knife Type

  // 1. Manually pull CS LOW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  // 2. Transmit
  HAL_SPI_Transmit(&hspi1, tx_buffer, sizeof(tx_buffer), 100);

  // 3. Manually pull CS HIGH
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

robot_hmi_command_t SPI_Poll_Main_Board(void){
  uint8_t tx_buffer[1] = {SPI_CMD_POLL_STATUS}; 
  uint8_t rx_buffer[1] = {0};
  
  // 1. Manually pull CS LOW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  // 2. Ask for status update
  HAL_SPI_Transmit(&hspi1, tx_buffer, sizeof(tx_buffer), 100);
  
  for(volatile int i = 0; i < 500; i++) {} // DELAY FOR PROCESSING
  
  // 3. Receive answer
  HAL_SPI_Receive(&hspi1, rx_buffer, sizeof(rx_buffer), 100);

  // 4. Manually pull CS HIGH
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  
  return (robot_hmi_command_t)rx_buffer[0]; 
}

/**
  * @brief Reads an Active-High button, safely debounces it, and plays a click sound
  */
uint8_t Button_Pressed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) 
    {
        HAL_Delay(50); 
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 125);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
        HAL_Delay(15); 
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) {} 
        return 1; 
    }
    return 0; 
}

/**
  * @brief Plays a musical 4-note arpeggio (C-Major) using PWM
  */
void Play_Startup_Tune(void)
{   
    uint16_t notes[] = { 1912, 0, 1912, 0, 2145, 0, 2409, 0, 2409, 0, 2145 };
    uint16_t durations[] = { 250, 240, 200, 360, 300, 150, 250, 240, 200, 360, 300 };
    int total_steps = sizeof(notes) / sizeof(notes[0]);

    for(int i = 0; i < total_steps; i++) {
        uint16_t current_arr = notes[i];
        if (current_arr == 0) {
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
            HAL_Delay(durations[i]);
        } else {
            __HAL_TIM_SET_AUTORELOAD(&htim4, current_arr);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, current_arr / 2); 
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
            HAL_Delay(durations[i]);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
            HAL_Delay(20);
        }
    }
}

/**
  * @brief Displays a startup splash screen for 5 seconds, then blacks out
  */
void Startup_Splash_Screen(void)
{
    for (uint32_t i = 0; i < (LCD_WIDTH * LCD_HEIGHT); i++) { fb[i] = L8_BLACK; }

    LTDC_LayerCfgTypeDef pLayerCfg1 = {0};
    uint16_t splash_x = (LCD_WIDTH / 2) - (SPLASH_WIDTH / 2)+50;
    uint16_t splash_y = (LCD_HEIGHT / 2) - (SPLASH_HEIGHT / 2);

    pLayerCfg1.WindowX0      = splash_x; 
    pLayerCfg1.WindowX1      = splash_x + SPLASH_WIDTH - 1; 
    pLayerCfg1.WindowY0      = splash_y;                         
    pLayerCfg1.WindowY1      = splash_y + SPLASH_HEIGHT - 1;            
    pLayerCfg1.PixelFormat   = LTDC_PIXEL_FORMAT_RGB565; 
    pLayerCfg1.FBStartAdress = (uint32_t)xcalibur_image; 
    pLayerCfg1.ImageWidth    = SPLASH_WIDTH; 
    pLayerCfg1.ImageHeight   = SPLASH_HEIGHT; 
    pLayerCfg1.Alpha         = 255; 
    pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA; 
    pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    
    HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1);
    HAL_LTDC_ConfigColorKeying(&hltdc, 0x000000, 1);
    HAL_LTDC_EnableColorKeying(&hltdc, 1);

    HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
    Play_Startup_Tune();
    HAL_Delay(3000);
    __HAL_LTDC_LAYER_DISABLE(&hltdc, 1);

    while (SPI_Poll_Main_Board() != USB_CONNECTED){}
    HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
    HAL_Delay(500);
    __HAL_LTDC_LAYER_ENABLE(&hltdc, 1);
}

/**
  * @brief Updates the dynamic text, arrows, and image!
  */
void Update_Tool_UI(uint8_t show_ui, const char* title, const uint8_t* image_array, uint8_t text_size)
{
    if (image_array != NULL) {
        LTDC_LayerCfgTypeDef pLayerCfg1 = {0};
        uint16_t center_x = (LCD_WIDTH / 2) - (IMAGE_WIDTH / 2);  
        uint16_t center_y = (LCD_HEIGHT / 2) - (IMAGE_HEIGHT / 2); 

        pLayerCfg1.WindowX0      = center_x; 
        pLayerCfg1.WindowX1      = center_x + IMAGE_WIDTH - 1; 
        pLayerCfg1.WindowY0      = center_y;                         
        pLayerCfg1.WindowY1      = center_y + IMAGE_HEIGHT - 1;            
        pLayerCfg1.PixelFormat   = LTDC_PIXEL_FORMAT_RGB565; 
        pLayerCfg1.FBStartAdress = (uint32_t)image_array; 
        pLayerCfg1.ImageWidth    = IMAGE_WIDTH; 
        pLayerCfg1.ImageHeight   = IMAGE_HEIGHT; 
        pLayerCfg1.Alpha         = 255; 
        pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA; 
        pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;

        HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1);
        HAL_LTDC_ConfigColorKeying(&hltdc, 0x000000, 1);
        HAL_LTDC_EnableColorKeying(&hltdc, 1);
        __HAL_LTDC_LAYER_ENABLE(&hltdc, 1); 
    } else {
        __HAL_LTDC_LAYER_DISABLE(&hltdc, 1);
    }

    L8_FillRect(0, 520, UI_WIDTH, 40, L8_BLACK);
    L8_FillRect(0, 380, UI_WIDTH, 40, L8_BLACK);

    uint16_t text_width = strlen(title) * 18;
    uint16_t start_x = (UI_WIDTH > text_width) ? (UI_WIDTH - text_width) / 2 : 5;
    L8_DrawString(start_x, 530, title, L8_WHITE, text_size);

    if (show_ui) {
        uint16_t left_tip_x = 60, left_tip_y = 400;
        L8_FillRect(left_tip_x, left_tip_y - 2, 45, 4, L8_WHITE); 
        for (int i=0; i < 18; i++) { 
            L8_DrawPixel(left_tip_x + i, left_tip_y - i, L8_WHITE); 
            L8_DrawPixel(left_tip_x + i, left_tip_y - i - 1, L8_WHITE); 
            L8_DrawPixel(left_tip_x + i, left_tip_y + i, L8_WHITE); 
            L8_DrawPixel(left_tip_x + i, left_tip_y + i + 1, L8_WHITE); 
        }
        
        uint16_t right_tip_x = 420, right_tip_y = 400;
        L8_FillRect(right_tip_x - 45, right_tip_y - 2, 45, 4, L8_WHITE);
        for (int i=0; i < 18; i++) { 
            L8_DrawPixel(right_tip_x - i, right_tip_y - i, L8_WHITE); 
            L8_DrawPixel(right_tip_x - i, right_tip_y - i - 1, L8_WHITE); 
            L8_DrawPixel(right_tip_x - i, right_tip_y + i, L8_WHITE); 
            L8_DrawPixel(right_tip_x - i, right_tip_y + i + 1, L8_WHITE); 
        }
    }
}

/**
  * @brief Static Framework (Layer 0 memory)
  */
void GUI_Framework(void)
{
  for (uint32_t i = 0; i < (LCD_WIDTH * LCD_HEIGHT); i++) { fb[i] = L8_BLACK; }
  L8_FillRect(0, 0, UI_WIDTH, BANNER_HEIGHT, L8_BLUE);
  L8_DrawString(60, 28, "XCALIBUR ARES", L8_WHITE, 3);
}

/**
  * @brief Draws a dynamic progress bar on Layer 0
  */
void L8_DrawProgressBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t percent) 
{
    if (percent > 100) percent = 100;
    L8_FillRect(x, y, width, 2, L8_WHITE); 
    L8_FillRect(x, y + height - 2, width, 2, L8_WHITE); 
    L8_FillRect(x, y, 2, height, L8_WHITE); 
    L8_FillRect(x + width - 2, y, 2, height, L8_WHITE); 

    uint16_t fill_width = ((width - 4) * percent) / 100;
    if (fill_width > 0) { L8_FillRect(x + 2, y + 2, fill_width, height - 4, L8_RED); }
    if (fill_width < (width - 4)) { L8_FillRect(x + 2 + fill_width, y + 2, (width - 4) - fill_width, height - 4, L8_BLACK); }
}

/**
  * @brief Prints a string using full ASCII table
  */
void L8_DrawString(uint16_t x, uint16_t y, const char* str, uint8_t colorIndex, uint8_t scale) 
{
    while (*str) {
        char c = *str;
        if (c < 32 || c > 126) { c = '?'; }
        uint8_t glyph_index = c - 32;
        for (int i = 0; i < 8; i++) {
            uint8_t row = font8x8_ascii[glyph_index][i];
            for (int j = 0; j < 8; j++) {
                if (row & (0x80 >> j)) { L8_FillRect(x + (j * scale), y + (i * scale), scale, scale, colorIndex); }
            }
        }
        x += (8 * scale) + 2; 
        str++;
    }
}

/**
  * @brief MPU Configuration to allow direct SRAM access for drawing
  */
void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};
    HAL_MPU_Disable();
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = INTERNAL_FB_ADDRESS; 
    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void L8_DrawPixel(uint16_t x, uint16_t y, uint8_t colorIndex)
{
    if (x >= UI_WIDTH || y >= UI_HEIGHT) return;
    uint16_t phys_x = y;
    uint16_t phys_y = (LCD_HEIGHT - 1) - x;
    fb[phys_y * LCD_WIDTH + phys_x] = colorIndex;
}

void L8_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t colorIndex)
{
    for (uint16_t i = 0; i < h; i++) {
        for (uint16_t j = 0; j < w; j++) { L8_DrawPixel(x + j, y + i, colorIndex); }
    }
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
  
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLL3.PLL3M = 4;        
  PeriphClkInitStruct.PLL3.PLL3N = 120;      
  PeriphClkInitStruct.PLL3.PLL3P = 2;        
  PeriphClkInitStruct.PLL3.PLL3Q = 2;        
  PeriphClkInitStruct.PLL3.PLL3R = 8;        
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }
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
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  
  if (HAL_LTDC_Init(&hltdc) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief Setup the LCD layer configuration
  */
void LCD_InitLayer(void)
{
    LTDC_LayerCfgTypeDef pLayerCfg = {0};
    pLayerCfg.WindowX0      = 0; 
    pLayerCfg.WindowX1      = LCD_WIDTH - 1; 
    pLayerCfg.WindowY0      = 0;                         
    pLayerCfg.WindowY1      = LCD_HEIGHT - 1;
    pLayerCfg.PixelFormat   = LTDC_PIXEL_FORMAT_L8; 
    pLayerCfg.FBStartAdress = INTERNAL_FB_ADDRESS; 
    pLayerCfg.ImageWidth    = LCD_WIDTH; 
    pLayerCfg.ImageHeight   = LCD_HEIGHT; 
    pLayerCfg.Alpha         = 255;          
    pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;

    if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK) { Error_Handler(); }

    static uint32_t Layer0_CLUT[4];
    Layer0_CLUT[L8_BLACK] = 0xFF000000; 
    Layer0_CLUT[L8_BLUE]  = 0xFF34BFFF; 
    Layer0_CLUT[L8_WHITE] = 0xFFFFFFFF; 
    Layer0_CLUT[L8_RED]   = 0xFFFF0000; 

    HAL_LTDC_ConfigCLUT(&hltdc, Layer0_CLUT, 4, 0); 
    HAL_LTDC_EnableCLUT(&hltdc, 0);                 
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
  
  // Software NSS ensures the clock can always fire freely
  hspi1.Init.NSS = SPI_NSS_SOFT; 
  
  // Slow clock for initial hardware testing
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; 
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

  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM4 Initialization Function
  */
static void MX_TIM4_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 199; 
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 249;    
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOD_CLK_ENABLE(); 
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; 
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4; 
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE(); 

  HAL_GPIO_WritePin(GPIOE, GREEN_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, RED_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DISP_GPIO_Port, DISP_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = RED_LED_Pin|GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SELECT_BTTN_Pin | DOWN_BTTN_Pin | UP_BTTN_Pin | LEFT_BTTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL; 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_BTTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RIGHT_BTTN_GPIO_Port, &GPIO_InitStruct);

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
  while (1) {
      HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
      HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
      for(volatile uint32_t i = 0; i < 20000000; i++) {} 
  }
}