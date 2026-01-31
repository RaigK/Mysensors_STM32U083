# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MySensors sensor node implementation for STM32U083RC microcontroller with RFM69 radio. This is a low-power temperature/humidity sensor using HDC1080 with battery monitoring, designed to communicate with a MySensors gateway.

## Build Commands

```bash
# Build the project
pio run

# Upload to device (requires ST-Link or STM32CubeProgrammer)
pio run -t upload

# Monitor serial output (115200 baud)
pio device monitor

# Clean build
pio run -t clean
```

## Architecture

### Library Override Mechanism

The `lib/MySensors_patch/` directory contains patched MySensors library files. PlatformIO's library dependency finder (LDF) loads these patches **before** the upstream MySensors library because local `lib/` has higher priority. The `library.json` marks it as a valid PlatformIO library.

Key patches:
- `hal/architecture/STM32/MyHwSTM32.cpp` - Complete STM32 hardware abstraction with low-power STOP mode sleep, RTC Alarm-based wake-up (uses STM32RTC library), GPIO interrupt wake-up, EEPROM emulation
- `hal/architecture/STM32/MyHwSTM32.h` - Header for the STM32 HAL implementation
- `hal/crypto/generic/drivers/AES/AES.h` - Fixes macro conflict between STM32 HAL's `AES` peripheral definition and MySensors' `AES` class (lines 39-41: `#ifdef AES #undef AES`)

### STM32U0 Compatibility Fixes

The STM32U0 series requires workarounds defined in `platformio.ini`:

- `RTC_ISR_INITS=RTC_ICSR_INITS` - U0 uses ICSR register instead of ISR for RTC initialization check
- `RTC_WKUP_IRQn=RTC_TAMP_IRQn` - U0 combines RTC wake-up into TAMP interrupt vector (uses `RTC_TAMP_IRQHandler`)
- `-include "include/stm32_aes_fix.h"` - Force-included before all headers

### Custom Board Definition

`boards/stm32u083rc.json` defines:
- Cortex-M0+ at 4MHz default clock (for low power)
- 256KB flash, 40KB RAM
- Uses variant `STM32U0xx/U073R(8-B-C)(I-T)_U083RC(I-T)` from STM32duino core

### Pin Configuration

Radio (RFM69 on SPI1):
- SCK: PA5, MISO: PA6, MOSI: PA7
- CS: PB6, IRQ: PA10

Sensor (HDC1080 on I2C2):
- SCL: PB10, SDA: PB11

Battery ADC: PA0

## Key Implementation Details

### Sleep Mode

The HAL implements true low-power sleep using STM32 STOP1 mode. The `hwSleep()` functions in MyHwSTM32.cpp:
1. Initialize RTC via STM32RTC library with LSI clock (32 kHz internal oscillator)
2. Configure RTC Alarm A for timed wake-up (adds sleep duration to current time)
3. Disable peripheral clocks (SPI1, I2C1/2, ADC) before entering STOP
4. Enter STOP mode with low-power regulator via `HAL_PWR_EnterSTOPMode()`
5. On wake: call `SystemClock_Config()` to restore clocks from HSI, re-enable peripherals

Wake-up sources: RTC Alarm A or GPIO interrupts (for radio IRQ).

**STM32U0-specific sleep details:**
- **IMPORTANT**: The RTC wake-up timer (WUTF) on STM32U0 does NOT work - the counter never decrements. This implementation uses RTC Alarm A instead, which works reliably.
- RTC Alarm uses EXTI line 17 (rising edge trigger)
- Internal wake-up line enabled via `PWR->CR3 |= PWR_CR3_EIWUL`
- Debug is disabled during sleep (`DBGMCU->CR = 0`) to allow true low power
- Uses STM32RTC library for simplified RTC management

### MySensors Configuration

Key defines in `src/main.cpp`:
- `MY_RADIO_RFM69` with `MY_RFM69_NEW_DRIVER`
- `MY_IS_RFM69HW` for high-power module
- `MY_RFM69_FREQUENCY RFM69_868MHZ`
- `MY_RFM69_CS_PIN PB6`, `MY_RFM69_IRQ_PIN PA10`
- `MY_NODE_ID 11` (static node ID)
- `MY_DEBUG` for serial debug output (comment out for lowest power)
- `MY_SMART_SLEEP_WAIT_DURATION_MS 0` - disable smart sleep for faster wake cycles
