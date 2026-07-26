/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "rms_amplitude.h"
#include "zero_cross.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FS            83900.0f  /* 84MHz / 1001 ≈ 83.9kHz */
#define LOCAL_N       256
#define MAX_PAGE      2
#define ADC_BUF_SIZE  1024
#define ZC_N          1024
#define V_SCALE       (1.0f / 1.1f)  /* ADC 参考电压校准 */

#define KEY0_PORT     GPIOE
#define KEY0_PIN      GPIO_PIN_4
#define KEY_UP_PORT   GPIOA
#define KEY_UP_PIN    GPIO_PIN_0
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint16_t adc_buf[ADC_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void KEY_Init(void);
static uint8_t KEY_Scan(void);
static void Show_Page1(float zc_freq, float rms_amp);
static void Show_Page2(float dc_v, float vpp_v, uint16_t raw0, uint16_t raw1, int zc_count);
static void MX_ADC_Init(void);
static void KEY_UP_Init(void);
static uint8_t KEY_UP_Scan(void);
static void USART2_Init(void);
static void USART2_Send(const char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Show_Float(uint16_t x, uint16_t y, float val, uint8_t size, uint16_t color)
{
    if (val < 0) { val = -val; lcd_show_char(x, y, '-', size, 0, color); x += size / 2; }
    uint32_t scaled = (uint32_t)(val * 100.0f + 0.5f);
    uint32_t int_part = scaled / 100;
    uint32_t dec_part = scaled % 100;

    uint16_t w = size * 2 + size / 2 + size;
    lcd_fill(x, y, x + w - 1, y + size - 1, WHITE);

    lcd_show_num(x, y, int_part, 2, size, color);
    lcd_show_char(x + size, y, '.', size, 0, color);
    lcd_show_xnum(x + size + size / 2, y, dec_part, 2, size, 0x80, color);
}

static void MX_ADC_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* TIM2: 84MHz / 1001 ≈ 83.9kHz */
    static TIM_HandleTypeDef htim2;
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);
    TIM_MasterConfigTypeDef sMaster = {0};
    sMaster.MasterOutputTrigger = TIM_TRGO_UPDATE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMaster);

    /* DMA2 Stream0 Channel0 */
    static DMA_HandleTypeDef hdma;
    hdma.Instance = DMA2_Stream0;
    hdma.Init.Channel = DMA_CHANNEL_0;
    hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma.Init.MemInc = DMA_MINC_ENABLE;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma.Init.Mode = DMA_CIRCULAR;
    hdma.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma);

    /* ADC1 */
    static ADC_HandleTypeDef hadc1;
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma);
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_SIZE);
}

static void KEY_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    gpio.Pin  = KEY0_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY0_PORT, &gpio);
}

static uint8_t KEY_Scan(void)
{
    static uint32_t last_time = 0;
    static uint8_t  last_state = 1;
    uint32_t now = HAL_GetTick();
    if (now - last_time < 50) return 0;
    uint8_t cur = (HAL_GPIO_ReadPin(KEY0_PORT, KEY0_PIN) == GPIO_PIN_RESET) ? 0 : 1;
    last_time = now;
    if (cur == 0 && last_state == 1) { last_state = 0; return 1; }
    last_state = cur;
    return 0;
}

static void KEY_UP_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin  = KEY_UP_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(KEY_UP_PORT, &gpio);
}

static uint8_t KEY_UP_Scan(void)
{
    static uint32_t last_time = 0;
    static uint8_t  last_state = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_time < 50) return 0;
    uint8_t cur = (HAL_GPIO_ReadPin(KEY_UP_PORT, KEY_UP_PIN) == GPIO_PIN_SET) ? 1 : 0;
    last_time = now;
    if (cur == 1 && last_state == 0) { last_state = 1; return 1; }
    last_state = cur;
    return 0;
}

static void USART2_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    USART2->BRR = 42000000 / 460800;      /* APB1 42MHz / 460800 ≈ 91 */
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void USART2_Send(const char *str)
{
    while (*str)
    {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *str++;
    }
}

static void Show_Page1(float zc_freq, float rms_amp)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 24, 24, "P1: Measure", BLACK);
    lcd_draw_line(10, 38, 230, 38, BLACK);

    lcd_show_string(20, 60,  80, 24, 24, "Freq:", BLACK);
    lcd_show_num(90, 60, (uint16_t)zc_freq, 5, 24, BLACK);
    lcd_show_string(155, 60, 40, 24, 24, "Hz", BLACK);

    lcd_show_string(20, 110, 80, 24, 24, "Amp:", BLACK);
    Show_Float(110, 110, rms_amp, 24, BLACK);
    lcd_show_string(185, 110, 40, 24, 24, "V", BLACK);

    lcd_show_string(10, 270, 220, 16, 16, "Press KEY0 -> Page2", BLACK);
}

static void Show_Page2(float dc_v, float vpp_v, uint16_t raw0, uint16_t raw1, int zc_count)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 16, 16, "P2: Debug", BLACK);
    lcd_draw_line(10, 28, 230, 28, BLACK);

    lcd_show_string(10, 40,  50, 16, 16, "DC:", BLACK);
    Show_Float(60, 40, dc_v, 16, BLACK);
    lcd_show_string(135, 40, 30, 16, 16, "V", BLACK);

    lcd_show_string(10, 65,  50, 16, 16, "Vpp:", BLACK);
    Show_Float(60, 65, vpp_v, 16, BLACK);
    lcd_show_string(135, 65, 30, 16, 16, "V", BLACK);

    lcd_show_string(10, 95,  50, 16, 16, "Raw0:", BLACK);
    lcd_show_num(60, 95, raw0, 5, 16, BLACK);

    lcd_show_string(10, 120, 50, 16, 16, "Raw1:", BLACK);
    lcd_show_num(60, 120, raw1, 5, 16, BLACK);

    lcd_show_string(10, 150, 50, 16, 16, "ZCc:", BLACK);
    lcd_show_num(60, 150, zc_count, 5, 16, BLACK);

    lcd_show_string(10, 270, 220, 16, 16, "Press KEY0 -> Page1", BLACK);
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  delay_init(168);
  led_init();
  lcd_init();
  KEY_Init();
  KEY_UP_Init();
  MX_ADC_Init();
  USART2_Init();

  uint8_t page = 1;
  lcd_clear(WHITE);

  /* USER CODE END 2 */

  while (1)
  {
    if (KEY_UP_Scan())
    {
        USART2_Send("COMMUNICATION TEST\r\n");
        for (int i = 0; i < 5; i++) { LED1_TOGGLE(); delay_ms(200); }
        LED1(0);
        delay_ms(500);   /* 防止连发 */
    }

    /* === 拷贝 ADC 数据到本地 === */
    uint16_t local_buf[LOCAL_N];
    memcpy(local_buf, adc_buf, LOCAL_N * sizeof(uint16_t));

    /* === DC / Vpp === */
    uint32_t raw_sum = 0;
    uint16_t raw_min = 4095, raw_max = 0;
    for (uint32_t i = 0; i < LOCAL_N; i++)
    {
        raw_sum += local_buf[i];
        if (local_buf[i] < raw_min) raw_min = local_buf[i];
        if (local_buf[i] > raw_max) raw_max = local_buf[i];
    }
    float dc_raw  = (float)raw_sum / (float)LOCAL_N;
    float dc_v    = dc_raw * 3.3f / 4096.0f * V_SCALE;
    float vpp_v   = (float)(raw_max - raw_min) * 3.3f / 4096.0f * V_SCALE;

    /* === 零交叉测频 === */
    static float32_t zc_buf[ZC_N];
    float32_t dc_zc = 0.0f;
    for (uint32_t i = 0; i < ZC_N; i++) dc_zc += (float32_t)adc_buf[i];
    dc_zc /= (float32_t)ZC_N;
    for (uint32_t i = 0; i < ZC_N; i++)
        zc_buf[i] = ((float32_t)adc_buf[i] - dc_zc) * 3.3f / 4096.0f;
    float zc_freq = ZeroCross_Freq(zc_buf, ZC_N, FS);
    int zc_count = ZeroCross_Count(zc_buf, ZC_N);
    /* 修正: ZeroCross_Freq 计算的是半周期频率, 直接用 Count 手算 */
    float zc_freq_fixed = (float)zc_count * 0.5f * FS / (float)ZC_N;

    /* === RMS 幅值 === */
    float rms_amp = Measuring_Sine_Amplitude(LOCAL_N, local_buf) * V_SCALE;

    /* === 显示 === */
    if (page == 1)
        Show_Page1(zc_freq_fixed, rms_amp);
    else
        Show_Page2(dc_v, vpp_v, local_buf[0], local_buf[1], zc_count);

    /* === 按键 === */
    if (KEY_Scan())
        page = page % MAX_PAGE + 1;

    LED0_TOGGLE();
    delay_ms(300);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    Error_Handler();
}

static void MX_NVIC_Init(void)
{
  HAL_NVIC_SetPriority(NonMaskableInt_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
