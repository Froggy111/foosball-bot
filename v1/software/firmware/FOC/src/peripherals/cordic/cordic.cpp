#include "cordic.hpp"

#include <stm32g4xx_hal.h>

#include "debug.hpp"
#include "error.hpp"

static CORDIC_HandleTypeDef handle;

void cordic::init(void) {
    __HAL_RCC_CORDIC_CLK_ENABLE();
    CORDIC_ConfigTypeDef config;

    handle.Instance = CORDIC;
    if (HAL_CORDIC_Init(&handle) != HAL_OK) {
        debug::fatal("Cordic initialisation failed");
        error::handler();
    }

    config.Function = CORDIC_FUNCTION_SINE;
    config.Precision = CORDIC_PRECISION_4CYCLES;
    config.Scale = CORDIC_SCALE_0;
    config.NbWrite = CORDIC_NBWRITE_1;
    config.NbRead = CORDIC_NBREAD_1;
    config.InSize = CORDIC_INSIZE_16BITS;
    config.OutSize = CORDIC_OUTSIZE_16BITS;

    if (HAL_CORDIC_Configure(&handle, &config) != HAL_OK) {
        debug::fatal("Cordic configuration failed");
        error::handler();
    }
}

cordic::SinCosVal cordic::sincos(float theta) {
    int16_t q15_theta = (int16_t)(theta * 32768.0f / M_PI);
    debug::log("q15_theta: %d", q15_theta);
    int32_t input_buffer = (int32_t)((uint32_t)q15_theta << 16) | 32768;
    volatile int32_t output_buffer;

    if (HAL_CORDIC_Calculate(&handle, &input_buffer, (int32_t*)&output_buffer,
                             1, HAL_MAX_DELAY) != HAL_OK) {
        error::handler();
    }
    debug::log("output: %d", output_buffer);

    int16_t q15_sin = (int16_t)(output_buffer & 0xFFFF);
    int16_t q15_cos = (int16_t)(output_buffer >> 16);

    debug::log("q15_sin: %d, q15_cos: %d", q15_sin, q15_cos);

    float sin = (float)q15_sin / 32768.0f;
    float cos = (float)q15_cos / 32768.0f;

    return SinCosVal{sin, cos};
}
