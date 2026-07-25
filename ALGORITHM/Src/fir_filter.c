/**
 * @file fir_filter.c
 * @brief 固定长度 CMSIS-DSP FIR 滤波器封装实现。
 */

#include "fir_filter.h"

static float32_t firStateF32[NUM_PER_CALL + FIR_EFFICIENT_NUM - 1];
static arm_fir_instance_f32 S_BP;
const float32_t firCoeffs32BP[FIR_EFFICIENT_NUM] =  /* 传递给 CMSIS-DSP 的固定 FIR 滤波器系数。 */
{
    -0.03723169863f,  0.08422418684f,  0.2265876085f, -0.3660466075f,
    -0.1893559098f,   0.6495268345f,  -0.1893559098f, -0.3660466075f,
     0.2265876085f,   0.08422418684f, -0.03723169863f
};



#define DC_AFTER_FILTER     3088

    // 初始化 CMSIS-DSP FIR 状态，并绑定滤波器系数。
void arm_emg_f32_filter_init(void)
{
    // 保存配置的抽头数量和每次处理的块长度。
    arm_fir_init_f32(&S_BP, FIR_EFFICIENT_NUM, (float32_t *)&firCoeffs32BP[0], &firStateF32[0], NUM_PER_CALL);

}

    // 对一块 NUM_PER_CALL 点输入执行 FIR 滤波。
void arm_emg_f32_filter(float32_t *dataInput, float32_t *dataOutput)
{
    arm_fir_f32(&S_BP, dataInput, dataOutput, NUM_PER_CALL);
}
