/**
 * @file ad9226.c
 * @brief AD9226 GPIO + TIM3 PWM driver (F407 port).
 *
 * Data lines : PB0-PB11  (12-bit parallel input)
 * Sample clk : PC6 = TIM3_CH1  (PWM output drives AD9226 CLK)
 *
 * Same pin scheme as the H743 reference (data PB0-PB11, clk PC6=TIM3_CH1).
 */
#include "ad9226.h"
#include "stm32f4xx_hal.h"

extern volatile uint8_t  adc_done;
extern volatile uint16_t adc_idx;
extern volatile uint8_t  active_buf;
extern uint16_t adc_buf0[];
extern uint16_t adc_buf1[];

static TIM_HandleTypeDef htim3;

void AD9226_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB0-PB11: 12-bit parallel data input */
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10| GPIO_PIN_11;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* PC6 = TIM3_CH1 PWM (sample clock output) */
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    /* F407 TIM3 clk = 84MHz. Period 419 -> 200kHz sample rate.
     * H743 used Period 119 on a faster clock; adjust as needed. */
    htim3.Init.Period            = 419;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 210;   /* ~50% duty */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 210);

    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void AD9226_Start(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint16_t raw;
    if (htim->Instance != TIM3) return;
    uint16_t *pbuf = active_buf ? adc_buf1 : adc_buf0;
    /* read low 12 bits of GPIOC as 12-bit parallel sample */
    raw = (uint16_t)(GPIOB->IDR & 0x0FFF);
    pbuf[adc_idx++] = raw;
    if (adc_idx >= AD9226_BUF_SIZE) {
        adc_idx    = 0;
        active_buf = (uint8_t)!active_buf;
        adc_done   = 1;
    }
}
