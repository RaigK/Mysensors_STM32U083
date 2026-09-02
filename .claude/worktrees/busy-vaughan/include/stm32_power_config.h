/**
 * STM32 Low Power Mode Configuration
 *
 * This header defines compile-time options for different power modes.
 * Include this before MySensors.h in your sketch.
 *
 * Run Modes:
 * - POWER_RUN_NORMAL:    16 MHz HSI, normal regulator (~3-5 mA)
 * - POWER_RUN_LOW_POWER: 2 MHz MSI, low-power regulator (~200-500 µA)
 *
 * Sleep Modes:
 * - POWER_SLEEP_SLEEP:      CPU halted, peripherals on, fast wake (~1 mA)
 * - POWER_SLEEP_LP_SLEEP:   Low-power sleep, must be in LPR mode (~100-200 µA)
 * - POWER_SLEEP_STOP0:      Stop mode 0, fast wake-up (~10-20 µA)
 * - POWER_SLEEP_STOP1:      Stop mode 1, lower power (~5-10 µA)
 * - POWER_SLEEP_STOP2:      Stop mode 2, lowest Stop power (~1-3 µA)
 * - POWER_SLEEP_STANDBY:    Standby, RAM lost, RTC retained (~300 nA)
 *
 * Usage in main.cpp:
 *   #define MY_STM32_RUN_MODE    POWER_RUN_LOW_POWER
 *   #define MY_STM32_SLEEP_MODE  POWER_SLEEP_STOP1
 *   #include "stm32_power_config.h"
 */

#ifndef STM32_POWER_CONFIG_H
#define STM32_POWER_CONFIG_H

// ======================== Run Mode Definitions ========================
#define POWER_RUN_NORMAL      0  // Normal run: 16 MHz HSI, normal regulator
#define POWER_RUN_LOW_POWER   1  // Low Power Run: 2 MHz MSI, LP regulator

// ======================== Sleep Mode Definitions ========================
#define POWER_SLEEP_SLEEP     0  // Sleep: CPU halted, peripherals running
#define POWER_SLEEP_LP_SLEEP  1  // Low Power Sleep: requires LPR mode
#define POWER_SLEEP_STOP0     2  // Stop 0: fast wake-up
#define POWER_SLEEP_STOP1     3  // Stop 1: lower power (default)
#define POWER_SLEEP_STOP2     4  // Stop 2: lowest Stop power
#define POWER_SLEEP_STANDBY   5  // Standby: RAM lost, lowest power

// ======================== Default Configuration ========================
#ifndef MY_STM32_RUN_MODE
#define MY_STM32_RUN_MODE     POWER_RUN_NORMAL
#endif

#ifndef MY_STM32_SLEEP_MODE
#define MY_STM32_SLEEP_MODE   POWER_SLEEP_STOP1
#endif

// ======================== Validation ========================
#if MY_STM32_SLEEP_MODE == POWER_SLEEP_LP_SLEEP && MY_STM32_RUN_MODE != POWER_RUN_LOW_POWER
#warning "LP Sleep requires Low Power Run mode. Forcing POWER_RUN_LOW_POWER."
#undef MY_STM32_RUN_MODE
#define MY_STM32_RUN_MODE POWER_RUN_LOW_POWER
#endif

// ======================== Clock Configuration ========================
#if MY_STM32_RUN_MODE == POWER_RUN_LOW_POWER
// Low Power Run: 2 MHz MSI
#define POWER_SYSCLK_FREQ     2000000UL
#define POWER_VOLTAGE_SCALE   PWR_REGULATOR_VOLTAGE_SCALE2
#else
// Normal Run: 16 MHz HSI
#define POWER_SYSCLK_FREQ     16000000UL
#define POWER_VOLTAGE_SCALE   PWR_REGULATOR_VOLTAGE_SCALE2
#endif

// ======================== Debug Helpers ========================
#if !defined(MY_DISABLED_SERIAL) && defined(MY_DEBUG)
#define POWER_DEBUG_ENABLED 1
#else
#define POWER_DEBUG_ENABLED 0
#endif

// Mode name strings for debug output
#if POWER_DEBUG_ENABLED
static inline const char* power_run_mode_name(void) {
#if MY_STM32_RUN_MODE == POWER_RUN_LOW_POWER
    return "Low Power Run (2 MHz)";
#else
    return "Normal Run (16 MHz)";
#endif
}

static inline const char* power_sleep_mode_name(void) {
#if MY_STM32_SLEEP_MODE == POWER_SLEEP_SLEEP
    return "Sleep";
#elif MY_STM32_SLEEP_MODE == POWER_SLEEP_LP_SLEEP
    return "LP Sleep";
#elif MY_STM32_SLEEP_MODE == POWER_SLEEP_STOP0
    return "Stop 0";
#elif MY_STM32_SLEEP_MODE == POWER_SLEEP_STOP1
    return "Stop 1";
#elif MY_STM32_SLEEP_MODE == POWER_SLEEP_STOP2
    return "Stop 2";
#elif MY_STM32_SLEEP_MODE == POWER_SLEEP_STANDBY
    return "Standby";
#else
    return "Unknown";
#endif
}
#endif // POWER_DEBUG_ENABLED

#endif // STM32_POWER_CONFIG_H
