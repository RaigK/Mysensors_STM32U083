#include "Arduino.h"
#include "stm32u0xx_hal.h"

extern "C" uint32_t millis(void)
{
    return HAL_GetTick();
}

/*
 * micros(): SysTick counts down from (SystemCoreClock/1000 - 1) to 0 each ms.
 * We combine the coarse ms count (uwTick) with the current SysTick VAL to
 * produce a monotonic 1-µs-resolution counter.
 */
extern "C" uint32_t micros(void)
{
    uint32_t ms, val, load;
    do {
        ms   = HAL_GetTick();
        val  = SysTick->VAL;
        load = SysTick->LOAD;
        /* Retry if a tick fired between reads. */
    } while (ms != HAL_GetTick());
    uint32_t elapsed_ticks = load - val;
    uint32_t us_in_ms = (elapsed_ticks * 1000u) / (load + 1);
    return ms * 1000u + us_in_ms;
}

extern "C" void delay(uint32_t ms)
{
    HAL_Delay(ms);
}

extern "C" void delayMicroseconds(uint32_t us)
{
    if (!us) return;
    uint32_t start = micros();
    while ((micros() - start) < us) { __asm__ volatile("nop"); }
}

extern "C" void yield(void) { /* cooperative hook — no-op */ }
