/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : F407 I2C2 Master - read from H743 slave
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
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define MAX_PAGE      3
#define KEY0_PORT     GPIOE
#define KEY0_PIN      GPIO_PIN_4
#define KEY_UP_PORT   GPIOA
#define KEY_UP_PIN    GPIO_PIN_0
#define I2C_ADDR      0x64     /* 0x32 << 1 */

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  version;
    uint8_t  byte_len;
    uint32_t sequence;
    uint32_t frequency_mhz;
    uint32_t vpp_uv;
    uint32_t vrms_uv;
    uint32_t sample_rate_hz;
    uint16_t range_mv;
    uint16_t status_flags;
    uint32_t checksum;
} I2C_Frame;
/* USER CODE END PD */

/* USER CODE BEGIN PV */
volatile I2C_Frame i2c_frame;
volatile uint8_t i2c_ready;
uint32_t last_seq;
float h7_rms, h7_fft, h7_freq;
volatile uint32_t h7_ok, h7_err;
I2C_HandleTypeDef hi2c2;
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
static void I2C2_Init(void);
static uint8_t I2C_Read_Frame(void);
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

static void I2C2_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &gpio);

    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = 100000;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c2);
}

static uint8_t I2C_Read_Frame(void)
{
    I2C_Frame tmp;
    if (HAL_I2C_Master_Receive(&hi2c2, I2C_ADDR, (uint8_t *)&tmp, sizeof(tmp), 50) != HAL_OK)
        return 0;

    if (tmp.magic != 0xC25A || tmp.version != 1 || tmp.byte_len != sizeof(tmp))
        return 0;

    uint32_t sum = 0;
    uint8_t *p = (uint8_t *)&tmp;
    for (int i = 0; i < 28; i++) sum += p[i];
    if (sum != tmp.checksum)
        return 0;

    if (tmp.sequence == last_seq)
        return 1;  /* 重复帧, 不算错 */
    last_seq = tmp.sequence;

    h7_rms   = tmp.vrms_uv / 1000000.0f;
    h7_fft   = tmp.vpp_uv  / 1000000.0f;
    h7_freq  = tmp.frequency_mhz / 1000.0f;
    h7_ok++;
    return 1;
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
    lcd_show_string(10, 10, 220, 16, 16, "P1: I2C Data", BLACK);
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
    lcd_show_num(80, 135, last_seq, 7, 16, BLACK);
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
    lcd_show_string(10, 95, 200, 16, 16, "I2C2 @100kHz 0x32", BLACK);
    lcd_show_string(10, 270, 220, 16, 16, "KEY0 -> P3", BLACK);
}

static void Show_Page3(void)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 16, 16, "P3: Frame Hex", BLACK);
    lcd_draw_line(10, 28, 230, 28, BLACK);
    uint8_t *p = (uint8_t *)&i2c_frame;
    for (int i = 0; i < 8 && i < 31; i++) {
        uint16_t x = 10 + (i % 4) * 55, y = 45 + (i / 4) * 25;
        lcd_show_char(x, y, "0123456789ABCDEF"[p[i] >> 4], 16, 0, BLACK);
        lcd_show_char(x + 10, y, "0123456789ABCDEF"[p[i] & 0x0F], 16, 0, BLACK);
    }
    lcd_show_string(10, 100, 80, 16, 16, "seq:", BLACK);
    lcd_show_num(50, 100, last_seq, 7, 16, BLACK);
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
  I2C2_Init();
  lcd_clear(WHITE);
  uint8_t page = 1;

  while (1)
  {
    if (page == 1) Show_Page1();
    else if (page == 2) Show_Page2();
    else Show_Page3();

    I2C_Read_Frame();

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
