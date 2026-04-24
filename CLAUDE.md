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

# Monitor serial output (COM9 at 115200 baud)
"C:/Users/raigk/.platformio/penv/Scripts/pio.exe" device monitor

# Clean build
"C:/Users/raigk/.platformio/penv/Scripts/pio.exe" run -t clean
```

Upload requires STM32CubeProgrammer installed at `C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/`. The upload command flashes to `0x08000000` via SWD and resets.

**Debugging with OpenOCD**: `stm32u0x.cfg` in the project root is a custom OpenOCD target config for the STM32U0 (based on the STM32L0 config with STM32L4x flash driver). Referenced by `board_debug.openocd_target = stm32u0x` in `platformio.ini`. Use PlatformIO's debug interface or `openocd -f stm32u0x.cfg` for live debugging. Note: debug sessions enable low-power debug mode via DBGMCU register in the examine-end event.

**Upload note**: The upload command uses `mode=UR` (Under Reset) at 480 kHz. This is required because `mode=NORMAL` cannot connect to the MCU while it is in Stop mode ("Unable to get core ID"). `mode=UR` keeps the MCU in reset during programming, then releases it. If `mode=UR` is unavailable, manually hold the RESET button while the upload command runs.

## Critical: Dual-File Sync

**MySensors.h `#include`s `MyHwSTM32.cpp` directly** (line 74) — it is not compiled as a separate library object. PlatformIO compiles the copy in `.pio/libdeps/stm32u083/MySensors/`, not the one in `lib/MySensors_patch/`. The `lib/MySensors_patch/` copy is the canonical source.

**After every edit to `lib/MySensors_patch/hal/architecture/STM32/MyHwSTM32.cpp`, sync it:**

```bash
cp 'C:/claude/Mysensors_STM32U083/lib/MySensors_patch/hal/architecture/STM32/MyHwSTM32.cpp' \
   'C:/claude/Mysensors_STM32U083/.pio/libdeps/stm32u083/MySensors/hal/architecture/STM32/MyHwSTM32.cpp'
```

## Architecture

### Library Override Mechanism

The `lib/MySensors_patch/` directory contains patched MySensors library files. PlatformIO's library dependency finder (LDF) loads these patches **before** the upstream MySensors library because local `lib/` has higher priority. The `library.json` marks it as a valid PlatformIO library. Do not rename or remove `library.json`.

Key patches:
- `hal/architecture/STM32/MyHwSTM32.cpp` - Complete STM32 hardware abstraction with low-power STOP mode sleep, RTC Alarm-based wake-up (uses STM32RTC library), GPIO interrupt wake-up, EEPROM emulation
- `hal/architecture/STM32/MyHwSTM32.h` - Header for the STM32 HAL implementation
- `hal/crypto/generic/drivers/AES/AES.h` - Fixes macro conflict between STM32 HAL's `AES` peripheral definition and MySensors' `AES` class (lines 39-41: `#ifdef AES #undef AES`). The crypto `.cpp` files under `hal/crypto/` are **excluded** from compilation via `library.json` (`-<hal/crypto/**>`), so only this header override matters — the upstream MySensors crypto implementation is used.

`library.json` build section:
- `srcFilter`: compiles `+<hal/**/*.cpp>` minus `-<hal/crypto/**>` — only HAL files, no crypto `.cpp`
- `flags`: `-include hal/architecture/STM32/MyHwSTM32.h` — force-includes the patched STM32 header before every compilation unit in the patch library, ensuring the header override takes effect even for files that don't explicitly include it

### STM32U0 Compatibility Fixes

The STM32U0 series requires workarounds defined in `platformio.ini`:

- `RTC_ISR_INITS=RTC_ICSR_INITS` - U0 uses ICSR register instead of ISR for RTC initialization check
- `RTC_WKUP_IRQn=RTC_TAMP_IRQn` - U0 combines RTC wake-up into TAMP interrupt vector (uses `RTC_TAMP_IRQHandler`)
- `-include "include/stm32_aes_fix.h"` - Force-included before all headers (the file itself is a placeholder; the actual `#undef AES` fix lives in the patched `lib/MySensors_patch/hal/crypto/generic/drivers/AES/AES.h`)
- `ARDUINO_ARCH_STM32` and `ARDUINO_NUCLEO_U083RC` - Required board identification defines
- `STM32U0xx` and `STM32U083xx` - Required for HAL peripheral register map selection
- `MY_STM32_RUN_MODE=2` and `MY_STM32_SLEEP_MODE=3` - Power mode numeric values mirroring `src/main.cpp` defines. Note: `stm32_power_config.h` comments say "MSI" for MEDIUM mode but the actual `SystemClock_Config()` implementation uses HSI 16 MHz ÷ AHB/4 = 4 MHz HCLK — HSI is used, not MSI

### Custom Board Definition

`boards/stm32u083rc.json` defines:
- Cortex-M0+ at 4MHz default clock (overridden by `SystemClock_Config()`)
- 256KB flash, 40KB RAM
- Uses variant `STM32U0xx/U073R(8-B-C)(I-T)_U083RC(I-T)` from STM32duino core

The `SystemClock_Config()` in `src/main.cpp` configures the actual runtime clock based on `MY_STM32_RUN_MODE`:
- **Normal**: 16 MHz HSI, voltage scale 1
- **Medium**: 16 MHz HSI with AHB/4 divider → 4 MHz HCLK, voltage scale 2 (serial-safe). Sets `__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI)` so HSI is selected immediately on Stop exit (default is MSI). Checks `PWR_CR1_LPR` (control bit) to detect active Low Power Run mode, not `PWR_SR2_REGLPF` (status bit).
- **Low Power Run**: 2 MHz MSI, LP regulator enabled

### Library Dependencies

Defined in `platformio.ini`:
- `MySensors` from Git `development` branch
- `STM32duino RTC` ^1.4.0 - RTC Alarm wake-up
- `ClosedCube HDC1080` ^1.3.2 - Temperature/humidity sensor

The Arduino core is pulled from `stm32duino/Arduino_Core_STM32` Git `main` branch (required for STM32U0 support).

### Pin Configuration

Radio (RFM69 on SPI1):
- SCK: PA5, MISO: PA6, MOSI: PA7 — also set as `PIN_SPI_SCK/MISO/MOSI` build flags in `platformio.ini` because the STM32duino core default SPI pin mapping for this variant may differ
- CS: PB6, IRQ: PA10
- SPI speed: 1 MHz (set via `MY_RFM69_SPI_SPEED`)

Sensor (HDC1080 on I2C2):
- SCL: PB10, SDA: PB11
- I2C clock: 10 kHz (reduced from default for reliability)

Status LED: PB0, referenced via `LED_PIN` define. Driven HIGH at the top of `loop()`, blinks at 1 Hz (6 × 150 ms toggles) during the first-boot 60 s parameter window, driven LOW before `sleep()`. PB0 is **excluded from the GPIO analog sweep** in `hwConfigureGpioLowPower()` so it stays driven LOW during STOP2 (prevents a pull-up from floating the pin HIGH and drawing ~0.5 mA during sleep). It is reinitialized as OUTPUT LOW in `hwRestoreAfterSleep()` before `loop()` drives it HIGH again. Any new output pin that must stay driven during sleep must be similarly excluded from the sweep and restored after wake.

**GPIO state during sleep** (set by `hwConfigureGpioLowPower()`):
- PA2/PA3 (USART2): analog — UART peripheral clock disabled, no leakage
- PA5/PA7 (SPI SCK/MOSI): **output LOW** — driven defined state to prevent spurious RFM69 activity from floating lines
- PA6 (SPI MISO): analog — input driven by radio, buffer disabled
- PB10/PB11 (I2C2 SCL/SDA): analog — disables ~40 kΩ internal pull-ups that would otherwise draw ~82 µA each (~165 µA total) in Stop2
- PA13/PA14 (SWD): always kept in AF mode regardless of `MY_DEBUG`
- PB0 (LED): excluded — stays driven LOW as OUTPUT

Battery ADC: PA0 (2:1 voltage divider, measures up to 6.6V). Uses factory-calibrated VREFINT to measure actual VDDA at runtime instead of assuming 3.3V. `readVDDA()` reads the internal VREFINT channel and computes VDDA from `VREFINT_CAL_ADDR`/`VREFINT_CAL_VREF` factory constants; `readBatteryVoltage()` uses that VDDA to scale the PA0 ADC reading. All ADC access goes through `readADC_U0()` (direct HAL, 12-bit) — `analogRead()` is not used.

Battery chemistry: **Lithium Thionyl Chloride (Li-SOCl2)**, 3.6V nominal. Battery percentage is mapped between `battMinMv` (cutoff, default 3100 mV) and `battMaxMv` (full, default 3600 mV). Both thresholds are runtime-adjustable via the gateway (children 6 and 7) and persisted to MySensors EEPROM. At or below the cutoff voltage (and above 2.5V as USB-only guard), the node sends a final battery voltage + 0% level and enters indefinite sleep to preserve remaining capacity — no further sensor or signal transmissions.

## Key Implementation Details

### Power Mode Configuration

Compile-time defines in `src/main.cpp` select run and sleep modes. Include `stm32_power_config.h` after setting defines. Current defaults: `POWER_RUN_MEDIUM` (4 MHz) and `POWER_SLEEP_STOP1`.

**Power mode defines exist in two places** — `src/main.cpp` (symbolic: `POWER_RUN_MEDIUM`, `POWER_SLEEP_STOP1`) and `platformio.ini` build flags (numeric: `-D MY_STM32_RUN_MODE=2 -D MY_STM32_SLEEP_MODE=3`). Both must be kept consistent. The numeric values come from `include/stm32_power_config.h` (POWER_RUN_MEDIUM=2, POWER_SLEEP_STOP1=3).

**Run Modes** (`MY_STM32_RUN_MODE`):
| Mode | Clock | Current | Use Case |
|------|-------|---------|----------|
| `POWER_RUN_NORMAL` | 16 MHz HSI | ~3-5 mA | Normal operation, fast processing |
| `POWER_RUN_MEDIUM` | 4 MHz (HSI/4) | ~1-2 mA | Serial-safe battery operation (current default) |
| `POWER_RUN_LOW_POWER` | 2 MHz MSI | ~200-500 µA | Lowest active power, breaks serial |

**Sleep Modes** (`MY_STM32_SLEEP_MODE`):
| Mode | Current | Wake Sources | Notes |
|------|---------|--------------|-------|
| `POWER_SLEEP_SLEEP` | ~1 mA | Any interrupt | Fast wake, peripherals on |
| `POWER_SLEEP_LP_SLEEP` | ~100-200 µA | Any interrupt | Requires LPR mode (auto-forced) |
| `POWER_SLEEP_STOP0` | ~10-20 µA | RTC, EXTI | Fast wake-up |
| `POWER_SLEEP_STOP1` | ~5-10 µA | RTC, EXTI | Good balance (current default) |
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
1. Initialize RTC via STM32RTC library with LSI clock (32 kHz); deactivate any stale WUT (`HAL_RTCEx_DeactivateWakeUpTimer()`) on first init — a WUT left armed from a previous firmware upload can prevent real sleep by waking the MCU immediately on every WUT pulse
2. Configure RTC Alarm A for timed wake-up (1-second resolution, sub-second values round up)
3. Call `hwPrepareSleep()`: DBGMCU handling (conditional on `MY_DEBUG`) → set EIWUL → clear stale WUT flags → **print diagnostics** → `hwConfigureGpioLowPower()` → disable peripheral clocks. **All `Serial.print` diagnostics must appear before `hwConfigureGpioLowPower()`** — after that call PA2 (UART TX) goes to analog mode and all serial output is silently lost.
4. Call `hwEnterSleepMode()` which selects mode based on `MY_STM32_SLEEP_MODE`
5. On wake: call `HAL_ResumeTick()` **before** `SystemClock_Config()` — the tick is frozen while SysTick is suspended and HAL functions inside `SystemClock_Config()` use `HAL_GetTick()` for timeouts, causing an infinite loop if tick is still frozen
6. Restore peripheral clocks, then restore (in order): USART2 (PA2/PA3 → AF7), SPI1 (PA5/PA6/PA7 → `GPIO_AF5_SPI1`), RFM69 CS (PB6), I2C2 (PB10/PB11 → AF4), ADC (PA0), PB0 (OUTPUT LOW — app code drives it HIGH on return from `sleep()`). USART2 must be restored first, before any `Serial.print` calls in `loop()`.

Wake-up sources: RTC Alarm A or GPIO interrupts (for radio IRQ). EIWUL (`PWR_CR3_EIWUL`, bit 15) is set as a belt-and-suspenders internal wakeup path; it is cleared after wake in `hwSleep()`.

**STM32U0-specific notes:**
- **On STM32U0, EXTI28 is the RTC Alarm A wakeup line in Stop mode** — `RTC_EXTI_LINE_ALARM_EVENT = EXTI_IMR1_IM28` (same line as WUT). EXTI28 is a "direct line": no RTSR/FTSR edge configuration is possible; `IMR28=1` arms the interrupt path. In Stop mode, EXTI28 **routes to `USART3_LPUART1_IRQn` (IRQ 29)**, NOT to `RTC_TAMP_IRQn` (IRQ 2). Stop mode entry uses **WFI** (not WFE). WFE was tried and hangs: `IMR28=1` arms the interrupt path but WFE requires `EMR28=1` (event mask) for EXTI events — without it no event ever reaches the CPU. `hwSleepInit()` enables IRQ 29 (`USART3_LPUART1_IRQn`) so WFI can wake on EXTI28. The existing `USART3_IRQHandler` in `uart.c` (strong symbol — do NOT redefine it in `MyHwSTM32.cpp`) runs on wake, finds no UART interrupt pending, and returns cleanly. `hwSleep()` then manually checks ALRAF and calls `HAL_RTC_AlarmIRQHandler()` to clear the flag and invoke `alarmCallback`. **Do NOT clear IMR28 before Stop mode** — it also disables the Alarm A interrupt, preventing wake-up. This matches the ST `PWR_STOP2_RTC` reference example which also uses WFI.
- **Do NOT use RTC WUT directly** — on STM32U0 the WUT fires EXTI28 and historically routed to `Default_Handler` (0.5 mA infinite loop) before `RTC_WKUP_IRQn=RTC_TAMP_IRQn` was set. Use Alarm A instead.
- **Stale WUT across firmware uploads**: The RTC is powered by VDD/VBAT and survives firmware uploads. If a previous firmware armed the WUT, `hwSleepInit()` calls `HAL_RTCEx_DeactivateWakeUpTimer()` on first init to prevent immediate-wake loops.
- **Library symbol conflicts**: `STM32duino RTC` defines `RTC_TAMP_IRQHandler` and `HAL_RTCEx_WakeUpTimerEventCallback` as **strong** (non-weak) symbols for STM32U0xx. Do NOT define either of these in `MyHwSTM32.cpp` — it will cause a linker error.
- DBGMCU handling in `hwPrepareSleep()` is **conditional on `MY_DEBUG`**: if defined, `HAL_DBGMCU_EnableDBGStopMode()` keeps SWD alive in STOP2 (~1 µA extra cost, enables upload without reset); if not defined, `DBGMCU->CR = 0` saves ~1 µA (production builds)
- PA13/PA14 are **always** kept in SWD AF mode in `hwConfigureGpioLowPower()` regardless of `MY_DEBUG` so the SWD pins themselves never go analog

### MySensors Configuration

Key defines in `src/main.cpp`:
- `MY_RADIO_RFM69` with `MY_RFM69_NEW_DRIVER`
- `MY_IS_RFM69HW` for high-power module
- `MY_RFM69_FREQUENCY RFM69_868MHZ`
- `MY_RFM69_CS_PIN PB6`, `MY_RFM69_IRQ_PIN PA10`, `MY_RFM69_IRQ_NUM PA10` (all three required by new driver)
- `MY_RFM69_TX_POWER_DBM (6)` - initial/max TX power in dBm before ATC adjusts down
- `MY_NODE_ID` — **not set** → auto-assigned by gateway on first connect (stored in MySensors EEPROM). Currently node 11 but may change if EEPROM is erased.
- `MY_DEBUG` for serial debug output (comment out for lowest power)
- `MY_DISABLED_SERIAL` - disables MySensors serial output; however, the `Serial.begin()` call and initial `=== BOOT ===` message in `before()` are **unconditional** and always execute regardless of this define
- `MY_SPLASH_SCREEN_DISABLED` - skips MySensors boot banner
- `MY_SMART_SLEEP_WAIT_DURATION_MS 0` - disable smart sleep for faster wake cycles
- `MY_SLEEP_TRANSPORT_RECONNECT_TIMEOUT_MS 0` - skip transport reconnect delay on wake
- `SLEEP_TIME_DEFAULT_MS 60000` - default sleep interval; runtime value `sleepTimeMs` is loaded from MySensors EEPROM via `eeLoad16(EE_INTERVAL_LO)` (positions 0–1, little-endian) on boot and overrides this default
- **`POWER_RUN_LOW_POWER` breaks serial**: at 2 MHz MSI, USART2 reinit after clock restore fails silently — no output on monitor. Use `POWER_RUN_NORMAL` or `POWER_RUN_MEDIUM` when `MY_DEBUG` or serial output is needed. Switch to `POWER_RUN_LOW_POWER` only with `MY_DISABLED_SERIAL`.

### before() Initialization

`before()` runs before MySensors initializes. It:
1. Enables DBGMCU clock and `HAL_DBGMCU_EnableDBGStopMode()` so SWD stays accessible immediately after boot
2. Switches Serial from the default LPUART1 to USART2 on `PA_2_ALT1` (TX) / `PA_3_ALT1` (RX) at 115200 baud — call `Serial.end()` first to release LPUART1
3. Prints `=== BOOT === (5s upload window)` and waits 5 seconds — a guaranteed SWD upload window after power-on, before the node enters sleep cycles. Remove the `delay(5000)` once firmware is stable.

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
| 5 | S_CUSTOM / V_VAR1 | Sleep interval (seconds) — gateway-adjustable, EEPROM pos 0–1, range 10–3600, default 60 |
| 6 | S_CUSTOM / V_VAR1 | Battery cutoff voltage (mV) — gateway-adjustable, EEPROM pos 2–3, range 2000–4200, default 3100 |
| 7 | S_CUSTOM / V_VAR1 | Battery full voltage (mV) — gateway-adjustable, EEPROM pos 4–5, range 2000–4200, default 3600 |

**Gateway SET commands** (node ID 11):
```
11;5;1;0;24;120    → set sleep interval to 120 s
11;6;1;0;24;2850   → set battery cutoff to 2.85 V
11;7;1;0;24;3700   → set battery full to 3.70 V
```
Values are validated and clamped in `receive()`, then RAM is updated immediately and a flag is set in the `pendingSave` bitfield struct. **EEPROM writes are deferred** — `receive()` never calls `saveState()` directly. Instead, `flushPendingSaves()` is called from `loop()` (after `wait()` returns) to perform the actual writes. This deferral is **safety-critical**: on the single-bank STM32U0, the CPU cannot fetch instructions from flash while a page erase is running; an RFM69 ISR vector fetch during that window stalls the CPU. All new gateway-adjustable parameters must follow this same pattern. EEPROM positions 0–5, little-endian uint16_t pairs via `eeLoad16()`/`eeSave16()`. On uninitialized flash (0xFF bytes), range checks detect first boot and fall back to defaults. **Do not call `send()` inside `receive()`** — a NACK on that send corrupts the MySensors transport state and causes the node to hang in subsequent `sendBatteryLevel()` or `wait()` calls. Each threshold is validated independently (range only) — cross-validation against the other threshold was removed because the gateway may send them in any order. On first boot, the node reports its current values to the controller and waits 60 seconds for gateway parameter SETs before entering the regular sleep cycle. The 60-second window is implemented as **60 × `wait(1000)` slices** (not a single `wait(60000)`) — a gateway SET+echo during a long `wait()` can leave the RFM69 driver in a state where `_process()` never returns, hanging `wait()` indefinitely; 1-second slices let each call time out cleanly and reset polling state. `flushPendingSaves()` is called after each slice to write any parameters received during that slice. Every subsequent wake ends with `wait(2000)` before `sleep()` to catch late gateway SET retries (the controller may retry delivery after receiving the sensor reports confirming the node is awake). **`sendBatteryLevel()` must be called BEFORE the 60-second wait**: the MySensors echo mechanism (auto-reply to gateway SETs) sends a transport frame after `receive()` returns during `wait()`, leaving transport state corrupted — calling `sendBatteryLevel()` or `send()` after the wait on first boot will hang the node.

### HardFault Debugging

A custom `HardFault_Handler` in `src/main.cpp` prints the stacked frame over serial before hanging. The naked wrapper passes MSP to `hard_fault_impl()`, which prints PC, LR, R0, and R12 in hex. This is the primary tool for diagnosing crashes — enable `MY_DEBUG` and watch serial output. The Cortex-M0+ stacks `{R0, R1, R2, R3, R12, LR, PC, xPSR}` on exception entry; `frame[6]` is PC and `frame[5]` is LR.

### Battery ADC Implementation Notes

`readADC_U0()` uses direct STM32 HAL calls (bypasses `analogRead()`):
- **STM32duino bug**: `analog.cpp` line ~1125 sets `AdcChannelConf.SamplingTime` to a raw cycle constant on STM32U0 (only `STM32G0xx` is excluded). On STM32U0, this field must be `ADC_SAMPLINGTIME_COMMON_1/2` — so `HAL_ADC_ConfigChannel()` fails silently and `analogRead()` returns 0.
- **Destructive DR read**: Reading `ADC1->DR` directly (e.g., for diagnostics) before `HAL_ADC_GetValue()` consumes the conversion result — on STM32U0/G0/C0, reading DR clears EOC and empties the result register. Only read DR once via `HAL_ADC_GetValue()`.
- **Calibration**: `HAL_ADCEx_Calibration_Start()` returns HAL_ERROR on STM32U0 — skipped. Factory trim (CALFACT) is loaded from OTP at power-on; no runtime calibration needed for battery monitoring.
- **Channel mapping**: PA0 = ADC1_IN4 (channel 4) on STM32U083 LQFP64. PC0–PC3 are IN0–IN3, so channels are offset — PA0 is NOT IN0.
- **Battery cutoff guard**: `batteryVoltage > 2.5f` prevents `sleep(0)` when running on USB only (PA0 reads ~0V when no battery is connected through the voltage divider).
- **ES0602 errata 2.1.2 — VREFINT_CAL unprogrammed on early devices**: Boards with date codes 333–345 (first Nucleo/Discovery batch, 2023) have `*VREFINT_CAL_ADDR == 0xFFFF` instead of the calibration value. If used raw in `readVDDA()`, this computes VDDA ≈ 115V and makes battery readings wildly wrong. Workaround in `readVDDA()`: check for `cal == 0xFFFF || cal == 0` and fall back to 3.0V.

### Known Silicon Errata (ES0602 Rev 3, February 2024)

Errata relevant to this firmware (all Rev A silicon):

- **2.1.2 VREFINT_CAL unprogrammed** (System, workaround applied): Early boards return `0xFFFF` from `VREFINT_CAL_ADDR`. Fixed in `readVDDA()` with a guard that falls back to 3.0V.
- **2.7.1 RTC Alarm flag stuck in debug** (RTC, no workaround): When a breakpoint is hit while the subsecond alarm condition is met, ALRAF is repeatedly set by hardware and cannot be cleared. Only affects debug sessions — production behavior is unaffected.
- **2.8.2 Spurious I2C BERR in master mode** (I2C, workaround: clear BERR flag): A spurious BERR flag may be set during I2C master transfers. The transfer continues normally. Wire library handles flag clearing. If unexpected I2C interrupts are observed, this is the likely cause.
- **2.11.1 SPI BSY stays high on disable in master transmit mode** (SPI, workaround: disable when TXE=1 and BSY=0): If SPI1 is disabled while TXE is low (data register full), BSY may stay high. HAL SPI deinit in `hwPrepareSleep()` waits for TXE before disabling, so this is low risk with the current implementation.
