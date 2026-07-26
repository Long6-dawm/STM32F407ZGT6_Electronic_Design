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
#include "arm_const_structs.h"
#include "rms_amplitude.h"
#include "iq_phase.h"
#include "zero_cross.h"
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
#define ZC_N          1024
#define ZC_PERIOD     100
#define MAX_PAGE      5
#define RMS_N         256
#define PH_N          1024

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
static void Show_Page3(float measured_hz, float expected_hz, float err_hz);
static void Show_Page4(float sine_amp, float sq_amp, float tri_amp);
static void Show_Page5(float measured_ph, float expected_ph, float err_ph);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Show_Float(uint16_t x, uint16_t y, float val, uint8_t size, uint16_t color)
{
    if (val < 0) { val = -val; lcd_show_char(x, y, '-', size, 0, color); x += size / 2; }
    uint32_t scaled = (uint32_t)(val * 100.0f + 0.5f);
    lcd_show_num(x, y, scaled / 100, 2, size, color);
    lcd_show_char(x + size, y, '.', size, 0, color);
    lcd_show_num(x + size + size / 2, y, scaled % 100, 2, size, color);
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

    if (cur == 0 && last_state == 1)
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
    Show_Float(110, 110, amp, 24, BLACK);
    lcd_show_string(185, 110, 40, 24, 24, "V", BLACK);

    lcd_show_string(20, 160, 80, 24, 24, "Phase:", BLACK);
    Show_Float(110, 160, phase, 24, BLACK);
    lcd_show_string(185, 160, 40, 24, 24, "rad", BLACK);

    lcd_show_string(10, 235, 220, 16, 16, "sin(PI/2)=", BLACK);
    Show_Float(110, 235, arm_sin_f32(1.5708f), 16, BLACK);

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
    lcd_show_string(10, 285, 220, 16, 16, "Press KEY0 -> Page3", BLACK);
}

static void Show_Page3(float measured_hz, float expected_hz, float err_hz)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 24, 24, "P3:Zero-Crossing", BLACK);
    lcd_draw_line(10, 38, 230, 38, BLACK);

    lcd_show_string(20, 55,  220, 20, 20, "Measured:", BLACK);
    lcd_show_num(20, 80, (uint16_t)measured_hz, 6, 24, BLACK);
    lcd_show_string(100, 80, 40, 24, 24, "Hz", BLACK);

    lcd_show_string(20, 125, 220, 20, 20, "Expected:", BLACK);
    lcd_show_num(20, 150, (uint16_t)expected_hz, 6, 24, BLACK);
    lcd_show_string(100, 150, 40, 24, 24, "Hz", BLACK);

    lcd_show_string(20, 195, 220, 20, 20, "Error:", BLACK);
    lcd_show_xnum(20, 220, (uint32_t)(err_hz * 10), 5, 24, 1, BLACK);
    lcd_show_string(100, 220, 40, 24, 24, "Hz", BLACK);

    lcd_show_string(10, 270, 220, 16, 16, "Press KEY0 -> Page4", BLACK);
}

static void Show_Page4(float sine_amp, float sq_amp, float tri_amp)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 24, 24, "P4:RMS Amplitude", BLACK);
    lcd_draw_line(10, 38, 230, 38, BLACK);

    lcd_show_string(20, 60,  80, 24, 24, "Sine:", BLACK);
    Show_Float(110, 60, sine_amp, 24, BLACK);
    lcd_show_string(185, 60, 40, 24, 24, "V", BLACK);

    lcd_show_string(20, 110, 80, 24, 24, "Square:", BLACK);
    Show_Float(110, 110, sq_amp, 24, BLACK);
    lcd_show_string(185, 110, 40, 24, 24, "V", BLACK);

    lcd_show_string(20, 160, 80, 24, 24, "Triang:", BLACK);
    Show_Float(110, 160, tri_amp, 24, BLACK);
    lcd_show_string(185, 160, 40, 24, 24, "V", BLACK);

    lcd_show_string(10, 270, 220, 16, 16, "Press KEY0 -> Page5", BLACK);
}

static void Show_Page5(float measured_ph, float expected_ph, float err_ph)
{
    lcd_clear(WHITE);
    lcd_show_string(10, 10, 220, 24, 24, "P5:I/Q Phase", BLACK);
    lcd_draw_line(10, 38, 230, 38, BLACK);

    lcd_show_string(20, 55,  220, 20, 20, "Measured:", BLACK);
    Show_Float(20, 80, measured_ph, 24, BLACK);
    lcd_show_string(100, 80, 40, 24, 24, "rad", BLACK);

    lcd_show_string(20, 125, 220, 20, 20, "Expected:", BLACK);
    Show_Float(20, 150, expected_ph, 24, BLACK);
    lcd_show_string(100, 150, 40, 24, 24, "rad", BLACK);

    lcd_show_string(20, 195, 220, 20, 20, "Error:", BLACK);
    Show_Float(20, 220, err_ph, 24, BLACK);
    lcd_show_string(100, 220, 40, 24, 24, "rad", BLACK);

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

  uint8_t page = 1;
  lcd_clear(WHITE);

  /* USER CODE END 2 */

  while (1)
  {
    /* === 生成测试信号: 800Hz正弦 === */
    float32_t fft_buf[FFT_N * 2];

    for (uint32_t i = 0; i < FFT_N; i++)
    {
        float32_t t = (float32_t)i / FS;
        float32_t val = 1000.0f * arm_sin_f32(2.0f * 3.14159265f * TEST_FREQ * t);
        fft_buf[2 * i]     = val;
        fft_buf[2 * i + 1] = 0.0f;
    }

    /* === CMSIS-DSP FFT === */
    arm_cfft_f32(&arm_cfft_sR_f32_len64, fft_buf, 0, 1);

    float32_t fft_mag[FFT_N / 2];
    float32_t max_mag = 0.0f;
    uint32_t  max_idx = 0;
    for (uint32_t i = 0; i < FFT_N / 2; i++)
    {
        float32_t sq = fft_buf[2 * i] * fft_buf[2 * i]
                     + fft_buf[2 * i + 1] * fft_buf[2 * i + 1];
        arm_sqrt_f32(sq, &fft_mag[i]);
        if (fft_mag[i] > max_mag)
        {
            max_mag = fft_mag[i];
            max_idx = i;
        }
    }

    uint16_t peak_freq = (uint16_t)(max_idx * FS / FFT_N);

    /* === 幅度测量 === */
    uint16_t adc_val[FFT_N];
    for (uint32_t i = 0; i < FFT_N; i++)
    {
        float32_t val = fft_buf[2 * i] + 2048.0f;
        if (val < 0) val = 0;
        if (val > 4095) val = 4095;
        adc_val[i] = (uint16_t)val;
    }
    float amp = Measuring_Sine_Amplitude(FFT_N, adc_val);

    /* === 相位测量 === */
    float32_t f32_vals[FFT_N];
    for (uint32_t i = 0; i < FFT_N; i++)
        f32_vals[i] = (fft_buf[2 * i]) / 1000.0f * 3.3f;
    float phase = CalPhase(TEST_FREQ, FS, FFT_N, f32_vals);

    /* === 零交叉测频 === */
    static float32_t zc_buf[ZC_N];
    float expected_freq = FS / (float)ZC_PERIOD;
    for (uint32_t i = 0; i < ZC_N; i++)
        zc_buf[i] = ((i % ZC_PERIOD) < (ZC_PERIOD / 2)) ? 1.0f : -1.0f;
    float measured_freq = ZeroCross_Freq(zc_buf, ZC_N, FS);
    float err_hz = measured_freq - expected_freq;

    /* === RMS 测幅: 正弦/方波/三角波 === */
    static uint16_t sine_buf[RMS_N], sq_buf[RMS_N], tri_buf[RMS_N];
    for (uint32_t i = 0; i < RMS_N; i++)
    {
        float t = (float)i / FS;
        sine_buf[i] = 2048 + (uint16_t)(1000.0f * arm_sin_f32(2.0f * 3.14159265f * 800.0f * t));
        sq_buf[i]  = ((i % 40) < 20) ? 3048 : 1048;
        float tri_val = (float)(i % 80);
        if (tri_val > 40) tri_val = 80 - tri_val;
        tri_buf[i] = 2048 + (uint16_t)((tri_val / 40.0f - 0.5f) * 2000.0f);
    }
    float sine_amp = Measuring_Sine_Amplitude(RMS_N, sine_buf);
    float sq_amp   = Measuring_Square_Amplitude(RMS_N, sq_buf);
    float tri_amp  = Measuring_Triangle_Amplitude(RMS_N, tri_buf);

    /* === 正交解调测相: 50Hz, 相位=0.5rad === */
    static float32_t ph_buf[PH_N];
    float ph_freq = 50.0f;
    float ph_expected = 0.5f;
    for (uint32_t i = 0; i < PH_N; i++)
        ph_buf[i] = arm_sin_f32(2.0f * 3.14159265f * ph_freq * (float32_t)i / FS + ph_expected);
    float ph_measured = CalPhase(ph_freq, FS, PH_N, ph_buf);
    float ph_err = ph_measured - ph_expected;

    /* === 按键翻页 === */
    if (KEY_Scan())
        page = page % MAX_PAGE + 1;

    /* === 显示当前页 === */
    if (page == 1)
        Show_Page1(peak_freq, amp, phase);
    else if (page == 2)
        Show_Page2(fft_mag, max_idx, peak_freq);
    else if (page == 3)
        Show_Page3(measured_freq, expected_freq, err_hz);
    else if (page == 4)
        Show_Page4(sine_amp, sq_amp, tri_amp);
    else
        Show_Page5(ph_measured, ph_expected, ph_err);

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
