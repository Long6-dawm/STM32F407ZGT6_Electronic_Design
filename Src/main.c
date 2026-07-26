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
#include "fft_n.h"
#include "rms_amplitude.h"
#include "iq_phase.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FFT_N         64
#define FS            51200.0f
#define TEST_FREQ     800.0f

#define KEY0_PORT     GPIOE
#define KEY0_PIN      GPIO_PIN_4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void KEY_Init(void);
static uint8_t KEY_Scan(void);
static void Show_Page1(uint16_t freq, float amp, float phase);
static void Show_Page2(float32_t *fft_mag, uint32_t max_idx, uint16_t peak_freq);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

    if (cur == 0 && last_state == 1)   /* 下降沿: 按下瞬间 */
    {
        last_state = 0;
        return 1;
    }
    last_state = cur;
    return 0;
}

static void Show_Page1(uint16_t freq, float amp, float phase)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 24, 24, "Page1: Measure", BLACK);
    lcd_draw_line(10, 38, 230, 38, BLACK);

    lcd_show_string(20, 60,  80, 24, 24, "Freq:", BLACK);
    lcd_show_num(110, 60, freq, 6, 24, BLACK);
    lcd_show_string(170, 60, 40, 24, 24, "Hz", BLACK);

    lcd_show_string(20, 110, 80, 24, 24, "Amp:", BLACK);
    lcd_show_xnum(110, 110, (uint32_t)(amp * 100), 5, 24, 1, BLACK);
    lcd_show_string(175, 110, 40, 24, 24, "V", BLACK);

    lcd_show_string(20, 160, 80, 24, 24, "Phase:", BLACK);
    lcd_show_xnum(110, 160, (uint32_t)(phase * 100), 5, 24, 1, BLACK);
    lcd_show_string(175, 160, 40, 24, 24, "rad", BLACK);

    lcd_show_string(10, 270, 220, 16, 16, "Press KEY0 -> Page2", BLACK);
}

static void Show_Page2(float32_t *fft_mag, uint32_t max_idx, uint16_t peak_freq)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 16, 16, "Page2: Spectrum", BLACK);

    lcd_draw_line(10, 240, 230, 240, BLACK);
    lcd_draw_line(10, 35, 10, 240, BLACK);

    float32_t max_val = fft_mag[max_idx];
    if (max_val < 1.0f) max_val = 1.0f;

    for (uint32_t i = 1; i < FFT_N / 2; i++)
    {
        uint16_t bar_h = (uint16_t)(fft_mag[i] / max_val * 200.0f);
        if (bar_h > 200) bar_h = 200;
        uint16_t bar_x = 11 + (uint16_t)(i * 218 / (FFT_N / 2));
        uint16_t bar_w = 218 / (FFT_N / 2);
        if (bar_w < 1) bar_w = 1;
        if (bar_h > 0)
            lcd_fill(bar_x, 240 - bar_h, bar_x + bar_w, 240, BLACK);
    }

    lcd_show_string(10, 265, 220, 16, 16, "Peak:", BLACK);
    lcd_show_num(60, 265, peak_freq, 6, 16, BLACK);
    lcd_show_string(130, 265, 40, 16, 16, "Hz", BLACK);
    lcd_show_string(10, 285, 220, 16, 16, "Press KEY0 -> Page1", BLACK);
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

  uint8_t page = 1;  /* 1=数据页, 2=频谱页 */
  lcd_clear(WHITE);

  /* USER CODE END 2 */

  while (1)
  {
    /* === 生成测试信号: 800Hz正弦 === */
    struct compx signal[FFT_N];
    float32_t adc_buf[FFT_N];

    for (uint32_t i = 0; i < FFT_N; i++)
    {
        float32_t t = (float32_t)i / FS;
        float32_t val = 1000.0f * arm_sin_f32(2.0f * 3.14159265f * TEST_FREQ * t);
        signal[i].real = val;
        signal[i].imag = 0.0f;
        adc_buf[i] = val + 2048.0f;  /* 模拟ADC偏置: 2048=1.65V */
    }

    /* === FFT 计算 === */
    InitTableFFT(FFT_N);
    cfft(signal, FFT_N);

    /* 幅度谱, 找峰值 */
    float32_t fft_mag[FFT_N / 2];
    float32_t max_mag = 0.0f;
    uint32_t  max_idx = 0;
    for (uint32_t i = 0; i < FFT_N / 2; i++)
    {
        float32_t sq = signal[i].real * signal[i].real
                     + signal[i].imag * signal[i].imag;
        arm_sqrt_f32(sq, &fft_mag[i]);
        if (fft_mag[i] > max_mag)
        {
            max_mag = fft_mag[i];
            max_idx = i;
        }
    }

    uint16_t peak_freq = (uint16_t)(max_idx * FS / FFT_N);

    /* === 幅度测量 (RMS -> 峰值 = sqrt(2)*RMS) === */
    /* 先转为 uint16_t 类型的 ADC 值 */
    uint16_t adc_val[FFT_N];
    for (uint32_t i = 0; i < FFT_N; i++)
    {
        float32_t v = adc_buf[i];
        if (v < 0) v = 0;
        if (v > 4095) v = 4095;
        adc_val[i] = (uint16_t)v;
    }
    float amp = Measuring_Sine_Amplitude(FFT_N, adc_val);

    /* === 相位测量 === */
    float32_t f32_vals[FFT_N];
    for (uint32_t i = 0; i < FFT_N; i++)
        f32_vals[i] = (adc_buf[i] - 2048.0f) / 4096.0f * 3.3f;
    float phase = CalPhase(TEST_FREQ, FS, FFT_N, f32_vals);

    /* === 按键检测, 翻页 === */
    if (KEY_Scan())
    {
        if (page == 1) page = 2;
        else           page = 1;
    }

    /* === 显示当前页 === */
    if (page == 1)
        Show_Page1(peak_freq, amp, phase);
    else
        Show_Page2(fft_mag, max_idx, peak_freq);

    LED0_TOGGLE();
    delay_ms(500);
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
