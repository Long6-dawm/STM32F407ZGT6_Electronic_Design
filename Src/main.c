/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : F407 Display-only: USART2 RX from H743
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define MAX_PAGE      3
#define KEY0_PORT     GPIOE
#define KEY0_PIN      GPIO_PIN_4
#define KEY_UP_PORT   GPIOA
#define KEY_UP_PIN    GPIO_PIN_0
#define RX_BUF_SIZE   200
/* USER CODE END PD */

/* USER CODE BEGIN PV */
volatile uint8_t rx_ready;
char  rx_buf[RX_BUF_SIZE];
uint8_t rx_idx;
float h7_rms, h7_fft, h7_freq;
uint32_t h7_frame;
volatile uint32_t h7_ok, h7_err;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void Show_Float(uint16_t x, uint16_t y, float v, uint8_t s, uint16_t c);
static void Show_Page1(void);
static void Show_Page2(void);
static void Show_Page3(void);
static void KEY_Init(void);
static uint8_t KEY_Scan(void);
static void KEY_UP_Init(void);
static uint8_t KEY_UP_Scan(void);
static void USART2_RX_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void Show_Float(uint16_t x, uint16_t y, float val, uint8_t size, uint16_t color)
{
    if (val < 0) { val = -val; lcd_show_char(x, y, '-', size, 0, color); x += size / 2; }
    uint32_t scaled = (uint32_t)(val * 100.0f + 0.5f);
    uint32_t ip = scaled / 100, dp = scaled % 100;
    uint16_t w = size * 2 + size / 2 + size;
    lcd_fill(x, y, x + w - 1, y + size - 1, WHITE);
    lcd_show_num(x, y, ip, 2, size, color);
    lcd_show_char(x + size, y, '.', size, 0, color);
    lcd_show_xnum(x + size + size / 2, y, dp, 2, size, 0x80, color);
}

void parse_frame(char *line)
{
    char *p;
    p = strstr(line, "RMS=");
    if (!p) { h7_err++; return; }
    float r = (float)atof(p + 4);
    p = strstr(line, "FFT=");
    if (!p) { h7_err++; return; }
    float f = (float)atof(p + 4);
    p = strstr(line, "Freq=");
    if (!p) { h7_err++; return; }
    float q = (float)atof(p + 5);
    uint32_t fr = 0;
    if (line[0] == '[')
        for (int i = 1; line[i] >= '0' && line[i] <= '9'; i++)
            fr = fr * 10 + (line[i] - '0');
    h7_rms = r; h7_fft = f; h7_freq = q; h7_frame = fr;
    h7_ok++;
}

static void USART2_RX_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);
    USART2->BRR = 42000000 / 460800;
    USART2->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void KEY_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    gpio.Pin = KEY0_PIN; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY0_PORT, &gpio);
}

static uint8_t KEY_Scan(void)
{
    static uint32_t lt = 0; static uint8_t ls = 1;
    uint32_t n = HAL_GetTick();
    if (n - lt < 50) return 0;
    uint8_t c = (HAL_GPIO_ReadPin(KEY0_PORT, KEY0_PIN) == GPIO_PIN_RESET) ? 0 : 1;
    lt = n;
    if (c == 0 && ls == 1) { ls = 0; return 1; }
    ls = c; return 0;
}

static void KEY_UP_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = KEY_UP_PIN; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(KEY_UP_PORT, &gpio);
}

static uint8_t KEY_UP_Scan(void)
{
    static uint32_t lt = 0; static uint8_t ls = 0;
    uint32_t n = HAL_GetTick();
    if (n - lt < 50) return 0;
    uint8_t c = (HAL_GPIO_ReadPin(KEY_UP_PORT, KEY_UP_PIN) == GPIO_PIN_SET) ? 1 : 0;
    lt = n;
    if (c == 1 && ls == 0) { ls = 1; return 1; }
    ls = c; return 0;
}

static void Show_Page1(void)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 16, 16, "P1: H7 Data", BLACK);
    lcd_draw_line(10, 28, 230, 28, BLACK);

    lcd_show_string(20, 45,  80, 16, 16, "RMS:", BLACK);
    Show_Float(80, 45, h7_rms, 16, BLACK);
    lcd_show_string(150, 45, 40, 16, 16, "V", BLACK);

    lcd_show_string(20, 75,  80, 16, 16, "FFT:", BLACK);
    Show_Float(80, 75, h7_fft, 16, BLACK);
    lcd_show_string(150, 75, 40, 16, 16, "V", BLACK);

    lcd_show_string(20, 105, 80, 16, 16, "Freq:", BLACK);
    lcd_show_num(80, 105, (uint16_t)h7_freq, 6, 16, BLACK);
    lcd_show_string(130, 105, 40, 16, 16, "Hz", BLACK);

    lcd_show_string(20, 135, 80, 16, 16, "Frm:", BLACK);
    lcd_show_num(80, 135, h7_frame, 7, 16, BLACK);

    lcd_show_string(10, 270, 220, 16, 16, "KEY0 -> P2", BLACK);
}

static void Show_Page2(void)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 16, 16, "P2: Status", BLACK);
    lcd_draw_line(10, 28, 230, 28, BLACK);
    lcd_show_string(10, 40, 80, 16, 16, "OK:", BLACK);
    lcd_show_num(60, 40, (uint32_t)h7_ok, 7, 16, BLACK);
    lcd_show_string(10, 65, 80, 16, 16, "ERR:", BLACK);
    lcd_show_num(60, 65, (uint32_t)h7_err, 7, 16, BLACK);
    lcd_show_string(10, 100, 200, 16, 16, "OK=parsed  ERR=bad", BLACK);
    lcd_show_string(10, 270, 220, 16, 16, "KEY0 -> P3", BLACK);
}

static void Show_Page3(void)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 16, 16, "P3: Raw", BLACK);
    lcd_draw_line(10, 28, 230, 28, BLACK);
    for (int i = 0; i < 8 && i < (int)rx_idx; i++) {
        uint8_t b = (uint8_t)rx_buf[i];
        uint16_t x = 10 + (i % 4) * 55, y = 45 + (i / 4) * 25;
        lcd_show_char(x, y, "0123456789ABCDEF"[b >> 4], 16, 0, BLACK);
        lcd_show_char(x + 10, y, "0123456789ABCDEF"[b & 0x0F], 16, 0, BLACK);
    }
    lcd_show_string(10, 100, 80, 16, 16, "idx:", BLACK);
    lcd_show_num(50, 100, rx_idx, 3, 16, BLACK);
    lcd_show_string(10, 270, 220, 16, 16, "KEY0 -> P1", BLACK);
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_NVIC_Init();
  delay_init(168);
  led_init();
  lcd_init();
  KEY_Init();
  KEY_UP_Init();
  USART2_RX_Init();
  lcd_clear(WHITE);
  uint8_t page = 1;

  while (1)
  {
    if (rx_ready) { rx_ready = 0; }
    if (page == 1) Show_Page1();
    else if (page == 2) Show_Page2();
    else Show_Page3();
    if (KEY_Scan()) page = page % MAX_PAGE + 1;
    if (KEY_UP_Scan()) { for (int i = 0; i < 3; i++) { LED1_TOGGLE(); delay_ms(100); } LED1(0); }
    LED0_TOGGLE();
    delay_ms(200);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_NVIC_Init(void)
{
  HAL_NVIC_SetPriority(NonMaskableInt_IRQn, 0, 0);
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
