/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : F407 AD9226 external 12-bit parallel ADC capture
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
#include "ad9226.h"
#include "rms_amplitude.h"
#include "zero_cross.h"
#include "dsp_analyzer.h"
#include "arm_const_structs.h"
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define MAX_PAGE      3
#define KEY0_PORT     GPIOE
#define KEY0_PIN      GPIO_PIN_4
#define KEY_UP_PORT   GPIOA
#define KEY_UP_PIN    GPIO_PIN_0
#define FS            600000.0f    /* TIM4 600kHz sample rate */
#define ZC_N          1024
#define FFT_N         1024
#define V_CAL         5.925f         /* AD9226 满量程校准系数 */
#define DBG_BUF_SIZE  128
/* USER CODE END PD */

/* USER CODE BEGIN PV */
uint16_t adc_buf0[AD9226_BUF_SIZE];
uint16_t adc_buf1[AD9226_BUF_SIZE];
volatile uint8_t  adc_done;
volatile uint16_t adc_idx;
volatile uint8_t  active_buf;

static uint16_t local_buf[AD9226_BUF_SIZE];
static float h7_rms, h7_fft, h7_freq, h7_freq_fft;
static uint32_t h7_frame;
static float sp_freq1, sp_amp1, sp_freq2, sp_amp2, sp_freq3, sp_amp3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
static char dbg_buf[DBG_BUF_SIZE];
static uint16_t dbg_len;
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
static void USART2_DMA_Init(void);
static void debug_send(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void USART2_DMA_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_2;    /* PA2 = USART2_TX */
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* DMA1 Stream6 Channel4 = USART2_TX */
    hdma_usart2_tx.Instance                 = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_usart2_tx);
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void dbg_char(char c)      { if (dbg_len < DBG_BUF_SIZE - 1) dbg_buf[dbg_len++] = c; }
static void dbg_str(const char *s){ while (*s && dbg_len < DBG_BUF_SIZE - 1) dbg_buf[dbg_len++] = *s++; }

static void dbg_num(uint32_t v)
{
    char t[12]; int i = 0;
    if (v == 0) { dbg_char('0'); return; }
    while (v) { t[i++] = '0' + (v % 10); v /= 10; }
    while (i) dbg_char(t[--i]);
}

static void debug_send(void)
{
    dbg_len = 0;
    dbg_char('['); dbg_num(h7_frame); dbg_str("] ");
    dbg_str("RMS=");
    dbg_num((uint32_t)(h7_rms * 1000.0f));          /* 单位 mV */
    dbg_str("mV FFT_amp=");
    dbg_num((uint32_t)(h7_fft * 1000.0f));          /* 单位 mV */
    dbg_str("mV Freq=");
    dbg_num((uint32_t)h7_freq);
    dbg_str("Hz FFT=");
    dbg_num((uint32_t)h7_freq_fft);
    dbg_str("Hz raw=");
    dbg_num(local_buf[0]); dbg_char(' ');
    dbg_num(local_buf[1]); dbg_char(' ');
    dbg_num(local_buf[2]);
    dbg_str(" F1=");
    dbg_num((uint32_t)sp_freq1); dbg_char('H');
    dbg_str(" A1=");
    dbg_num((uint32_t)sp_amp1); dbg_char('m');
    dbg_str(" F2=");
    dbg_num((uint32_t)sp_freq2); dbg_char('H');
    dbg_str(" A2=");
    dbg_num((uint32_t)sp_amp2); dbg_char('m');
    dbg_str("\r\n");
    dbg_buf[dbg_len] = '\0';

    while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {}
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)dbg_buf, dbg_len);
}

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

static void Process_Frame(void)
{
    uint16_t *src = active_buf ? adc_buf0 : adc_buf1;
    memcpy(local_buf, src, AD9226_BUF_SIZE * sizeof(uint16_t));

    h7_rms  = Measuring_Sine_Amplitude(AD9226_BUF_SIZE, local_buf) * V_CAL;
    h7_fft  = h7_rms;  /* 占位: FFT 幅度后续接入 */

    float32_t dc = 0.0f;
    for (int i = 0; i < ZC_N; i++) dc += (float)local_buf[i];
    dc /= ZC_N;
    static float32_t zc_buf[ZC_N];
    for (int i = 0; i < ZC_N; i++)
        zc_buf[i] = ((float)local_buf[i] - dc) * 3.3f / 4096.0f;
    int zc = ZeroCross_Count(zc_buf, ZC_N);
    h7_freq = (float)zc * FS / (float)ZC_N;

    /* === FFT 测频 + 测幅 === */
    static float32_t fft_buf[FFT_N * 2];
    for (int i = 0; i < FFT_N; i++)
    {
        fft_buf[2 * i]     = (float)local_buf[i] - dc;
        fft_buf[2 * i + 1] = 0.0f;
    }
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_buf, 0, 1);
    float32_t max_mag = 0.0f;
    uint32_t  max_idx = 0;
    for (int i = 1; i < FFT_N / 2; i++)
    {
        float32_t sq = fft_buf[2*i]*fft_buf[2*i] + fft_buf[2*i+1]*fft_buf[2*i+1];
        float32_t mag;
        arm_sqrt_f32(sq, &mag);
        if (mag > max_mag) { max_mag = mag; max_idx = i; }
    }
    h7_freq_fft = (float)max_idx * FS / (float)FFT_N;
    /* 单边谱峰值幅度 → 电压 (假设 VREF=3.3V) */
    h7_fft = max_mag * 2.0f / (float)FFT_N * 3.3f / 4096.0f * V_CAL;

    /* === DSP 频谱分析 (RFFT 4096 + Rife 插值 + 谐波) === */
    static float32_t adc_float[AD9226_BUF_SIZE];
    for (int i = 0; i < AD9226_BUF_SIZE; i++)
        adc_float[i] = (float)local_buf[i];
    DSP_Analyzer_Process(adc_float, &sp_freq1, &sp_amp1, &sp_freq2, &sp_amp2, &sp_freq3, &sp_amp3);

    h7_frame++;
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
    lcd_show_string(10, 10, 220, 16, 16, "P1: Spectrum", BLACK);
    lcd_draw_line(10, 28, 230, 28, BLACK);

    lcd_show_string(20, 38,  80, 16, 16, "RMS:", BLACK);
    Show_Float(80, 38, h7_rms, 16, BLACK);
    lcd_show_string(150, 38, 40, 16, 16, "V", BLACK);

    lcd_show_string(20, 66,  80, 16, 16, "F1:", BLACK);
    lcd_show_num(60, 66, (uint16_t)sp_freq1, 6, 16, BLACK);
    lcd_show_string(110, 66, 40, 16, 16, "Hz", BLACK);
    Show_Float(140, 66, sp_amp1 / 1000.0f, 16, BLACK);
    lcd_show_string(200, 66, 30, 16, 16, "V", BLACK);

    lcd_show_string(20, 94,  80, 16, 16, "F2:", BLACK);
    lcd_show_num(60, 94, (uint16_t)sp_freq2, 6, 16, BLACK);
    lcd_show_string(110, 94, 40, 16, 16, "Hz", BLACK);
    Show_Float(140, 94, sp_amp2 / 1000.0f, 16, BLACK);
    lcd_show_string(200, 94, 30, 16, 16, "V", BLACK);

    lcd_show_string(20, 122, 80, 16, 16, "F3:", BLACK);
    lcd_show_num(60, 122, (uint16_t)sp_freq3, 6, 16, BLACK);
    lcd_show_string(110, 122, 40, 16, 16, "Hz", BLACK);
    Show_Float(140, 122, sp_amp3 / 1000.0f, 16, BLACK);
    lcd_show_string(200, 122, 30, 16, 16, "V", BLACK);

    lcd_show_string(20, 154, 80, 16, 16, "Frm:", BLACK);
    lcd_show_num(80, 154, h7_frame, 7, 16, BLACK);
    lcd_show_string(10, 270, 220, 16, 16, "KEY0 -> P2", BLACK);
}

static void Show_Page2(void)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 16, 16, "P2: Status", BLACK);
    lcd_draw_line(10, 28, 230, 28, BLACK);
    lcd_show_string(10, 40, 160, 16, 16, "ADC @1MHz", BLACK);
    lcd_show_string(10, 65, 160, 16, 16, "Data PC0-PC11", BLACK);
    lcd_show_string(10, 90, 160, 16, 16, "CLK PB6=TIM4CH1", BLACK);
    lcd_show_string(10, 270, 220, 16, 16, "KEY0 -> P3", BLACK);
}

static void Show_Page3(void)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 16, 16, "P3: Raw[0..7]", BLACK);
    lcd_draw_line(10, 28, 230, 28, BLACK);
    for (int i = 0; i < 8; i++) {
        uint16_t b = local_buf[i];
        uint16_t x = 10 + (i % 4) * 55, y = 45 + (i / 4) * 25;
        lcd_show_num(x, y, b, 4, 16, BLACK);
    }
    lcd_show_string(10, 100, 160, 16, 16, "idx:", BLACK);
    lcd_show_num(50, 100, adc_idx, 5, 16, BLACK);
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

  AD9226_Init();
  AD9226_Start();
  USART2_DMA_Init();

  lcd_clear(WHITE);
  uint8_t page = 1;

  while (1)
  {
    if (adc_done)
    {
        adc_done = 0;
        Process_Frame();
        debug_send();
        AD9226_Resume();   /* 突发模式: 处理完重新采样下一帧 */
    }

    if (page == 1) Show_Page1();
    else if (page == 2) Show_Page2();
    else Show_Page3();

    if (KEY_Scan()) page = page % MAX_PAGE + 1;
    if (KEY_UP_Scan()) { for (int i = 0; i < 3; i++) { LED1_TOGGLE(); delay_ms(100); } LED1(0); }
    LED0_TOGGLE();
    delay_ms(100);
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
