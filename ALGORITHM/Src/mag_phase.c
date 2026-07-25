/**
 * @file mag_phase.c
 * @brief I/Q 测相模块附带的旧版幅度测量辅助函数。
 *
 * 实际实现已迁移至 rms_amplitude.c，此处仅保留编译单元以避免链接时
 * 出现重复符号。调用者应改为包含 rms_amplitude.h。
 */

#include "mag_phase.h"
#include "rms_amplitude.h"
