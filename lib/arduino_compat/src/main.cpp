/*
 * Arduino-compat entry points.
 *
 *   init()            — called by MySensors' MyMainSTM32.cpp::premain() (ctor 101).
 *                       Our responsibility: HAL_Init + SystemClock_Config.
 *   initVariant()     — weak, board-variant hook (empty).
 *   serialEventRun()  — weak, Arduino Serial-poll hook (empty).
 *   SystemClock_Config — weak default; user main.cpp may override.
 *   SysTick_Handler   — always strong, calls HAL_IncTick.
 *   main()            — weak fallback for Phase 1–3 non-MySensors sketches;
 *                       MySensors' MyMainSTM32.cpp provides the strong main().
 */

#include "Arduino.h"

extern "C" void SysTick_Handler(void) { HAL_IncTick(); }

extern "C" __attribute__((weak)) void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {};
    RCC_ClkInitTypeDef clk = {};

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_OFF;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}

extern "C" void init(void)
{
    HAL_Init();
    SystemClock_Config();
}

extern "C" __attribute__((weak)) void initVariant(void)    { }
extern "C" __attribute__((weak)) void serialEventRun(void) { }

extern "C" __attribute__((weak)) int main(void)
{
    init();
    setup();
    for (;;) { loop(); }
}

extern "C" __attribute__((weak)) void Error_Handler(void)
{
    __disable_irq();
    for (;;) { }
}
