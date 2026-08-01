/**
 * @file ad9226.c
 * @brief AD9226 GPIO + TIM4 PWM driver (F407 port).
 *
 * Data lines : PC0-PC11  (12-bit parallel input)
 * Sample clk : PB6 = TIM4_CH1  (PWM output drives AD9226 CLK)
 */
#include "ad9226.h"
#include "stm32f4xx_hal.h"

extern volatile uint8_t  adc_done;
extern volatile uint16_t adc_idx;
extern volatile uint8_t  active_buf;
extern uint16_t adc_buf0[];
extern uint16_t adc_buf1[];

TIM_HandleTypeDef htim4;

void AD9226_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC0-PC11: 12-bit parallel data input */
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10| GPIO_PIN_11;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* PB6 = TIM4_CH1 PWM (sample clock output) */
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_6;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    /* F407 TIM4 clk = 84MHz. Period 419 -> 200kHz sample rate. */
    htim4.Init.Period            = 419;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim4);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 210;   /* ~50% duty */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 210);

    HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void AD9226_Start(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint16_t raw;
    if (htim->Instance != TIM4) return;
    uint16_t *pbuf = active_buf ? adc_buf1 : adc_buf0;
    /* read low 12 bits of GPIOC as 12-bit parallel sample */
    raw = (uint16_t)(GPIOC->IDR & 0x0FFF);
    pbuf[adc_idx++] = raw;
    if (adc_idx >= AD9226_BUF_SIZE) {
        adc_idx    = 0;
        active_buf = (uint8_t)!active_buf;
        adc_done   = 1;
    }
}
