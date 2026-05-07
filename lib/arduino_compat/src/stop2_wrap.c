/*
 * Linker --wrap of HAL_PWR_EnterSTOPMode → enter STOP2 instead of STOP0.
 *
 * The MySensors cube-env upstream MyHwSTM32.cpp hardcodes the call
 *   HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
 * and ST's HAL implementation in turn writes LPMS=0 (Stop 0) regardless of
 * the regulator argument. We can't edit the libdeps copy durably, so we use
 * `-Wl,--wrap=HAL_PWR_EnterSTOPMode` (added in platformio.ini cube env) to
 * redirect every call site to this wrapper, which enters Stop 2 — the lowest
 * Stop mode on STM32U0 still compatible with RTC wake and RAM retention.
 *
 * Stop 2 caveats: most peripherals are off, only LP/RTC/IWDG keep running.
 * Our RTC alarm / WUT wake works in Stop 2. RFM69 IRQ on PA10 is an EXTI
 * line which also wakes from Stop 2.
 */

#include "stm32u0xx_hal.h"

void __wrap_HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry)
{
    (void)Regulator;
    HAL_PWREx_EnterSTOP2Mode(STOPEntry);
}
