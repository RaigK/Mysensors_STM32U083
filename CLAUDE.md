# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MySensors sensor node implementation for STM32U083RC microcontroller with RFM69 radio. This is a low-power temperature/humidity sensor using HDC1080 with battery monitoring, designed to communicate with a MySensors gateway.

## Build Commands

PlatformIO CLI is not in PATH — always use the full path:

```bash
# Build the project
"C:/Users/raigk/.platformio/penv/Scripts/pio.exe" run

# Upload to device (uses STM32CubeProgrammer CLI via ST-Link SWD)
"C:/Users/raigk/.platformio/penv/Scripts/pio.exe" run -t upload

# Monitor serial output (COM16 at 115200 baud)
"C:/Users/raigk/.platformio/penv/Scripts/pio.exe" device monitor

# Clean build
"C:/Users/raigk/.platformio/penv/Scripts/pio.exe" run -t clean
```

Upload requires STM32CubeProgrammer installed at `C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/`. The upload command flashes to `0x08000000` via SWD and resets.

**Upload troubleshooting**: When the MCU is in STOP2 sleep, the ST-Link cannot connect ("Unable to get core ID"). The user must **hold the reset button** during upload. First attempts often fail — just retry with reset held.

## Architecture

### Library Override Mechanism

The `lib/MySensors_patch/` directory contains patched MySensors library files. PlatformIO's library dependency finder (LDF) loads these patches **before** the upstream MySensors library because local `lib/` has higher priority. The `library.json` marks it as a valid PlatformIO library. Do not rename or remove `library.json`.

Key patches:
- `hal/architecture/STM32/MyHwSTM32.cpp` - Complete STM32 hardware abstraction with low-power STOP mode sleep, RTC Alarm-based wake-up (uses STM32RTC library), GPIO interrupt wake-up, EEPROM emulation
- `hal/architecture/STM32/MyHwSTM32.h` - Header for the STM32 HAL implementation
- `hal/crypto/generic/drivers/AES/AES.h` - Fixes macro conflict between STM32 HAL's `AES` peripheral definition and MySensors' `AES` class (lines 39-41: `#ifdef AES #undef AES`)
- `hal/crypto/generic/MyCryptoGeneric.cpp`, `hal/crypto/generic/drivers/AES/AES.cpp`, `AES_config.h` - Crypto driver files included to ensure the patched AES.h is used

### STM32U0 Compatibility Fixes

The STM32U0 series requires workarounds defined in `platformio.ini`:

- `RTC_ISR_INITS=RTC_ICSR_INITS` - U0 uses ICSR register instead of ISR for RTC initialization check
- `RTC_WKUP_IRQn=RTC_TAMP_IRQn` - U0 combines RTC wake-up into TAMP interrupt vector (uses `RTC_TAMP_IRQHandler`)
- `-include "include/stm32_aes_fix.h"` - Force-included before all headers
- `ARDUINO_ARCH_STM32` and `ARDUINO_NUCLEO_U083RC` - Required board identification defines

### Custom Board Definition

`boards/stm32u083rc.json` defines:
- Cortex-M0+ at 4MHz default clock (overridden by `SystemClock_Config()`)
- 256KB flash, 40KB RAM
- Uses variant `STM32U0xx/U073R(8-B-C)(I-T)_U083RC(I-T)` from STM32duino core

The `SystemClock_Config()` in `src/main.cpp` configures the actual runtime clock based on `MY_STM32_RUN_MODE`:
- **Normal**: 16 MHz HSI, voltage scale 2
- **Low Power Run**: 2 MHz MSI, LP regulator enabled

### Library Dependencies

Defined in `platformio.ini`:
- `MySensors` from Git `development` branch
- `STM32duino RTC` ^1.4.0 - RTC Alarm wake-up
- `ClosedCube HDC1080` ^1.3.2 - Temperature/humidity sensor

The Arduino core is pulled from `stm32duino/Arduino_Core_STM32` Git `main` branch (required for STM32U0 support).

### Pin Configuration

Radio (RFM69 on SPI1):
- SCK: PA5, MISO: PA6, MOSI: PA7
- CS: PB6, IRQ: PA10
- SPI speed: 1 MHz (set via `MY_RFM69_SPI_SPEED`)

Sensor (HDC1080 on I2C2):
- SCL: PB10, SDA: PB11

Battery ADC: PA0 (2:1 voltage divider, measures up to 6.6V)

## Key Implementation Details

### Power Mode Configuration

Compile-time defines in `src/main.cpp` select run and sleep modes. Include `stm32_power_config.h` after setting defines. Defaults: `POWER_RUN_NORMAL` (16 MHz) and `POWER_SLEEP_STOP1`.

**Run Modes** (`MY_STM32_RUN_MODE`):
| Mode | Clock | Current | Use Case |
|------|-------|---------|----------|
| `POWER_RUN_NORMAL` | 16 MHz HSI | ~3-5 mA | Normal operation, fast processing |
| `POWER_RUN_LOW_POWER` | 2 MHz MSI | ~200-500 µA | Battery operation, slow processing OK |

**Sleep Modes** (`MY_STM32_SLEEP_MODE`):
| Mode | Current | Wake Sources | Notes |
|------|---------|--------------|-------|
| `POWER_SLEEP_SLEEP` | ~1 mA | Any interrupt | Fast wake, peripherals on |
| `POWER_SLEEP_LP_SLEEP` | ~100-200 µA | Any interrupt | Requires LPR mode (auto-forced) |
| `POWER_SLEEP_STOP0` | ~10-20 µA | RTC, EXTI | Fast wake-up |
| `POWER_SLEEP_STOP1` | ~5-10 µA | RTC, EXTI | Default, good balance |
| `POWER_SLEEP_STOP2` | ~1-3 µA | RTC, EXTI | Lowest Stop power |
| `POWER_SLEEP_STANDBY` | ~300 nA | RTC, WKUP pins | RAM lost, system resets |

**Validation**: If `POWER_SLEEP_LP_SLEEP` is selected without `POWER_RUN_LOW_POWER`, the build auto-forces Low Power Run mode with a warning.

Example configuration:
```cpp
#define MY_STM32_RUN_MODE    POWER_RUN_LOW_POWER   // 2 MHz for battery life
#define MY_STM32_SLEEP_MODE  POWER_SLEEP_STOP2     // Lowest practical power
#include "stm32_power_config.h"
```

### Sleep Implementation Details

The `hwSleep()` functions in `MyHwSTM32.cpp` handle all sleep modes:
1. Initialize RTC via STM32RTC library with LSI clock (32 kHz)
2. Configure RTC Alarm A for timed wake-up (1-second resolution, sub-second values round up)
3. Call `hwPrepareSleep()` to disable peripheral clocks (SPI1, I2C1/2, ADC) and set unused GPIOs to analog mode
4. Call `hwEnterSleepMode()` which selects mode based on `MY_STM32_SLEEP_MODE`
5. On wake: restore clocks via `SystemClock_Config()`, re-enable peripherals, restore GPIO pin modes

Wake-up sources: RTC Alarm A or GPIO interrupts (for radio IRQ).

**STM32U0-specific notes:**
- **IMPORTANT**: RTC wake-up timer (WUTF) does NOT work on STM32U0 - use RTC Alarm A instead
- RTC Alarm uses EXTI line 17 (rising edge trigger)
- Internal wake-up line: `PWR->CR3 |= PWR_CR3_EIWUL`
- Debug disabled during sleep: `DBGMCU->CR = 0`

### MySensors Configuration

Key defines in `src/main.cpp`:
- `MY_RADIO_RFM69` with `MY_RFM69_NEW_DRIVER`
- `MY_IS_RFM69HW` for high-power module
- `MY_RFM69_FREQUENCY RFM69_868MHZ`
- `MY_RFM69_CS_PIN PB6`, `MY_RFM69_IRQ_PIN PA10`
- `MY_NODE_ID 11` (static node ID)
- `MY_DEBUG` for serial debug output (comment out for lowest power)
- `MY_DISABLED_SERIAL` - define to disable serial entirely for lowest power
- `MY_SMART_SLEEP_WAIT_DURATION_MS 0` - disable smart sleep for faster wake cycles

### ATC Signal Reporting

Requires `MY_SIGNAL_REPORT_ENABLED` and `MY_RFM69_ATC_TARGET_RSSI_DBM (-70)` defines. Use `transportGetSignalReport()` after `send()` calls:
- `SR_RX_RSSI` - works reliably, measures signal from ACK packets
- `SR_TX_POWER_LEVEL` - works, shows current ATC-adjusted TX power in dBm
- `SR_TX_RSSI` - **do not use**, returns 127 without gateway-side ATC support
- Uses `S_SOUND` / `V_LEVEL` sensor types for controller compatibility (`S_CUSTOM` / `V_CUSTOM` may not display values in some controllers)

### Sensor Channels

| Child ID | Type | Description |
|----------|------|-------------|
| 0 | S_TEMP | Temperature (HDC1080) |
| 1 | S_HUM | Humidity (HDC1080) |
| 2 | S_MULTIMETER | Battery voltage |
| 3 | S_SOUND | RX RSSI (dBm) |
| 4 | S_SOUND | TX Power level (dBm) |
