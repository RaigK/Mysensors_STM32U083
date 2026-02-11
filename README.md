# MySensors STM32U083RC Sensor Node

Low-power MySensors sensor node for STM32U083RC microcontroller with RFM69 radio, HDC1080 temperature/humidity sensor, and battery monitoring.

## Features

- **Ultra-low power**: Stop 2 sleep mode (~2-5 µA), Low Power Run mode (2 MHz)
- **Configurable power modes**: Compile-time selection of run and sleep modes
- **Temperature & Humidity**: HDC1080 sensor via I2C
- **Battery monitoring**: ADC with VREFINT-calibrated reference, 2:1 voltage divider
- **RFM69HW radio**: 868 MHz, high-power module support
- **ATC signal reporting**: RSSI and TX power level with automatic transmission control
- **MySensors compatible**: Works with any MySensors gateway/controller

## Hardware

| Component | Connection |
|-----------|------------|
| **MCU** | STM32U083RC (Cortex-M0+, 256KB Flash, 40KB RAM) |
| **Radio** | RFM69HW on SPI1 (PA5/PA6/PA7), CS=PB6, IRQ=PA10 |
| **Sensor** | HDC1080 on I2C2 (PB10=SCL, PB11=SDA) |
| **Battery** | 2:1 voltage divider on PA0 (ADC, VREFINT-calibrated) |

## Build & Upload

Requires [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Upload (ST-Link)
pio run -t upload

# Monitor serial (115200 baud)
pio device monitor

# Clean
pio run -t clean
```

## Power Mode Configuration

Edit `src/main.cpp` to select power modes:

```cpp
#define MY_STM32_RUN_MODE    POWER_RUN_LOW_POWER   // 2 MHz MSI
#define MY_STM32_SLEEP_MODE  POWER_SLEEP_STOP2     // ~2-5 µA sleep
#include "stm32_power_config.h"
```

### Run Modes

| Mode | Clock | Current | Description |
|------|-------|---------|-------------|
| `POWER_RUN_NORMAL` | 16 MHz HSI | ~3-5 mA | Fast processing |
| `POWER_RUN_LOW_POWER` | 2 MHz MSI | ~200-500 µA | Battery optimized |

### Sleep Modes

| Mode | Current | Description |
|------|---------|-------------|
| `POWER_SLEEP_SLEEP` | ~1 mA | CPU halted, fast wake |
| `POWER_SLEEP_LP_SLEEP` | ~100-200 µA | Low-power sleep |
| `POWER_SLEEP_STOP0` | ~10-20 µA | Stop mode, fast wake |
| `POWER_SLEEP_STOP1` | ~5-10 µA | Stop mode, balanced |
| `POWER_SLEEP_STOP2` | ~2-5 µA | Lowest practical power |
| `POWER_SLEEP_STANDBY` | ~300 nA | RAM lost on wake |

### Lowest Power Configuration

For minimum current consumption:

```cpp
#define MY_STM32_RUN_MODE    POWER_RUN_LOW_POWER
#define MY_STM32_SLEEP_MODE  POWER_SLEEP_STOP2
// #define MY_DEBUG                    // Disable debug
#define MY_DISABLED_SERIAL              // Disable serial
```

## Project Structure

```
├── src/main.cpp                 # Application code, power config
├── include/
│   └── stm32_power_config.h     # Power mode definitions
├── lib/MySensors_patch/         # Patched MySensors HAL for STM32
│   └── hal/architecture/STM32/
│       ├── MyHwSTM32.cpp        # Sleep modes, GPIO optimization
│       └── MyHwSTM32.h
├── boards/stm32u083rc.json      # Custom board definition
└── platformio.ini               # Build configuration
```

## STM32U0 Notes

- **RTC Wake-up Timer broken**: Uses RTC Alarm A instead (works reliably)
- **Register differences**: Requires `RTC_ISR_INITS=RTC_ICSR_INITS` define
- **GPIO optimization**: Unused pins set to analog mode to reduce leakage
- **ADC resolution**: STM32duino defaults to 10-bit; must call `analogReadResolution(12)` for 12-bit
- **ADC reference**: Uses factory-calibrated VREFINT to measure actual VDDA at runtime instead of assuming 3.3V

## MySensors Configuration

Key defines in `src/main.cpp`:

```cpp
#define MY_SIGNAL_REPORT_ENABLED
#define MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY RFM69_868MHZ
#define MY_RFM69_ATC_TARGET_RSSI_DBM (-70)
#define MY_NODE_ID 11
```

### Sensor Channels

| Child ID | Type | Description |
|----------|------|-------------|
| 0 | S_TEMP | Temperature (HDC1080) |
| 1 | S_HUM | Humidity (HDC1080) |
| 2 | S_MULTIMETER | Battery voltage |
| 3 | S_SOUND | RX RSSI (dBm) |
| 4 | S_SOUND | TX Power level (dBm) |

## License

MySensors library: GPL v2. See [MySensors](https://www.mysensors.org/).
