/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2025 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * DESCRIPTION
 * Temperature/Humidity sensor using HDC1080 with battery monitoring
 * for STM32U083RC
 *
 */

// ======================== Power Mode Configuration ========================
// Run modes: POWER_RUN_NORMAL (16 MHz HSI), POWER_RUN_MEDIUM (4 MHz MSI, serial-safe),
//            POWER_RUN_LOW_POWER (2 MHz MSI, requires MY_DISABLED_SERIAL)
// Sleep modes: POWER_SLEEP_SLEEP, POWER_SLEEP_LP_SLEEP, POWER_SLEEP_STOP0,
//              POWER_SLEEP_STOP1, POWER_SLEEP_STOP2, POWER_SLEEP_STANDBY
#define MY_STM32_RUN_MODE    POWER_RUN_MEDIUM      // 4 MHz MSI — serial-safe, ~4× less than 16 MHz
#define MY_STM32_SLEEP_MODE  POWER_SLEEP_STOP1     // STOP1
#include "stm32_power_config.h"

// Enable debug prints (comment out for lowest power)
#define MY_DEBUG
#define MY_SPLASH_SCREEN_DISABLED
// #define MY_DISABLED_SERIAL
#define MY_SMART_SLEEP_WAIT_DURATION_MS 0
#define MY_SLEEP_TRANSPORT_RECONNECT_TIMEOUT_MS 0

// Enable signal report functionalities
#define MY_SIGNAL_REPORT_ENABLED

// Enable and select radio type attached
#define MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY RFM69_868MHZ
#define MY_RFM69_TX_POWER_DBM (6)
#define MY_RFM69_ATC_TARGET_RSSI_DBM (-70)
#define MY_RFM69_CS_PIN PB6
#define MY_RFM69_IRQ_PIN PA10
#define MY_RFM69_IRQ_NUM PA10
// MY_NODE_ID not defined → auto-assign from gateway (stored in EEPROM by MySensors)
#define MY_RFM69_SPI_SPEED (1*1000000ul)

#include <MySensors.h>
#include <Wire.h>
#include "ClosedCube_HDC1080.h"
// I2C2 pins for HDC1080
#define I2C2_SDA_PIN PB11
#define I2C2_SCL_PIN PB10

// Default sleep interval between reports (milliseconds); overridden by gateway via EEPROM
#define SLEEP_TIME_DEFAULT_MS 60000UL  // 60 seconds

// Status LED on PB0 — on during active, 1 Hz blink during first-boot 60s wait, off in sleep
#define LED_PIN PB0

// Child IDs
#define CHILD_ID_TEMP     0
#define CHILD_ID_HUM      1
#define CHILD_ID_BATT     2
#define CHILD_ID_RSSI     3
#define CHILD_ID_TX_POWER 4
#define CHILD_ID_INTERVAL 5  // Gateway-adjustable sleep interval (seconds,  V_VAR1)
#define CHILD_ID_BATT_MIN 6  // Gateway-adjustable battery cutoff voltage (mV, V_VAR1)
#define CHILD_ID_BATT_MAX 7  // Gateway-adjustable battery full    voltage (mV, V_VAR1)

// MySensors user-state EEPROM positions for persistent config.
// loadState(pos) / saveState(pos, val) operate on single bytes; uint16_t values
// are stored as two consecutive bytes (little-endian).
// Note: MySensors also writes to EEPROM on first boot (auto node ID, parent ID,
// controller config). EEPROM only erases a flash page on GC (page full), not on
// every write, so multiple writes per boot are safe in practice.
#define EE_INTERVAL_LO  0   // sleep interval seconds, low  byte
#define EE_INTERVAL_HI  1   // sleep interval seconds, high byte
#define EE_BATT_MIN_LO  2   // battery cutoff mV, low  byte
#define EE_BATT_MIN_HI  3   // battery cutoff mV, high byte
#define EE_BATT_MAX_LO  4   // battery full   mV, low  byte
#define EE_BATT_MAX_HI  5   // battery full   mV, high byte

// Default Li-SOCl2 battery thresholds
#define BATT_MIN_DEFAULT_MV 3100  // 3.1 V cutoff (above 2.8 V LDO dropout)
#define BATT_MAX_DEFAULT_MV 3600  // 3.6 V fully charged

// Battery voltage ADC pin
#define BATTERY_ADC_PIN PA0  // ADC1_IN4, LQFP64 pin 14
// Voltage divider ratio: 2:1 (100k/100k), battery=3.2V → PA0=1.6V
#define VOLTAGE_DIVIDER_RATIO 2.0
// ADC resolution (12-bit)
#define ADC_MAX_VALUE 4095
// VDDA measured at runtime via VREFINT factory calibration (see readVDDA())

// HDC1080 sensor
ClosedCube_HDC1080 hdc1080;

// Runtime adjustable parameters (loaded from EEPROM at boot)
static uint32_t sleepTimeMs = SLEEP_TIME_DEFAULT_MS;
static uint16_t battMinMv   = BATT_MIN_DEFAULT_MV;
static uint16_t battMaxMv   = BATT_MAX_DEFAULT_MV;
static bool firstLoop = true;

// Pending EEPROM saves from receive() — actual flash write deferred to loop() to avoid
// triggering a flash page erase while the RFM69 ISR could fire.  On the single-bank
// STM32U0 the CPU cannot fetch instructions from flash while a page erase is running;
// any interrupt vector fetch during that window stalls (and may hang) the CPU.
static struct { bool interval : 1; bool battMin : 1; bool battMax : 1; } pendingSave = {};


// EEPROM helpers — MySensors saveState/loadState work per-byte
static uint16_t eeLoad16(uint8_t posLo)
{
	return (uint16_t)loadState(posLo) | ((uint16_t)loadState(posLo + 1) << 8);
}
static void eeSave16(uint8_t posLo, uint16_t val)
{
	saveState(posLo,     (uint8_t)(val & 0xFF));
	saveState(posLo + 1, (uint8_t)(val >> 8));
}

// MySensors messages
MyMessage msgTemp(CHILD_ID_TEMP,     V_TEMP);
MyMessage msgHum(CHILD_ID_HUM,       V_HUM);
MyMessage msgBatt(CHILD_ID_BATT,     V_VOLTAGE);
MyMessage msgRSSI(CHILD_ID_RSSI,     V_LEVEL);
MyMessage msgTxPower(CHILD_ID_TX_POWER, V_LEVEL);
MyMessage msgInterval(CHILD_ID_INTERVAL, V_VAR1);
MyMessage msgBattMin(CHILD_ID_BATT_MIN,  V_VAR1);
MyMessage msgBattMax(CHILD_ID_BATT_MAX,  V_VAR1);

// Custom system clock configuration with power mode support
// Supports: Normal Run (16 MHz HSI), Medium Run (4 MHz MSI), Low Power Run (2 MHz MSI)
extern "C" void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

#if MY_STM32_RUN_MODE == POWER_RUN_LOW_POWER
	// ==================== Low Power Run Mode (2 MHz MSI) ====================
	// Exit Low Power Run mode first if we're in it (required before clock changes)
	if (READ_BIT(PWR->SR2, PWR_SR2_REGLPF)) {
		HAL_PWREx_DisableLowPowerRunMode();
	}

	// Voltage scaling range 2 (required for LPR, valid up to 16 MHz)
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

	// Enable MSI at 2 MHz (range 5) and LSI for RTC
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;  // 2 MHz
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	// Use MSI as system clock (2 MHz)
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}

	// Enter Low Power Run mode (must be ≤ 2 MHz)
	HAL_PWREx_EnableLowPowerRunMode();

#elif MY_STM32_RUN_MODE == POWER_RUN_MEDIUM
	// ==================== Medium Run Mode (4 MHz HCLK from 16 MHz HSI / 4) ====================
	// Keep HSI as the SYSCLK source (avoids MSI switch issues on STM32U0) and divide
	// the AHB bus by 4 → HCLK = PCLK1 = 4 MHz.  Serial-safe: 115200 baud BRR = 35,
	// error < 1 %.  SystemCoreClock (HCLK) reflects the actual CPU frequency.
	//
	// Use LPR control bit (not REGLPF status bit) to detect Low Power Run mode.
	// REGLPF is set after ANY Stop mode exit (LP regulator was active in Stop), so
	// checking it here would call DisableLowPowerRunMode() on every wake-up — and
	// that function waits with HAL_GetTick() which is frozen after Stop exit (SysTick
	// interrupt was suspended), causing an infinite loop. LPR is only set when we
	// explicitly entered Low Power Run mode (2 MHz), which MEDIUM mode never does.
	if (READ_BIT(PWR->CR1, PWR_CR1_LPR)) {
		HAL_PWREx_DisableLowPowerRunMode();
	}

	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

	// Enable HSI (16 MHz) and LSI for RTC — same oscillators as NORMAL mode
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	// After Stop mode exit, STM32U0 defaults to MSI as system clock (STOPWUCK=0).
	// We are on HSI, so set STOPWUCK=1 to select HSI as the Stop wakeup clock.
	// This ensures that after any Stop exit, SYSCLK = HSI = 16 MHz immediately,
	// so HCLK = 4 MHz and SystemClock_Config() is effectively a no-op on wake-up.
	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);

	// SYSCLK = HSI 16 MHz, HCLK = SYSCLK / 4 = 4 MHz, PCLK1 = 4 MHz
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;   // 16 / 4 = 4 MHz HCLK
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}

#else
	// ==================== Normal Run Mode (16 MHz HSI) ====================
	// Check LPR control bit, not REGLPF status bit (see MEDIUM mode comment above).
	if (READ_BIT(PWR->CR1, PWR_CR1_LPR)) {
		HAL_PWREx_DisableLowPowerRunMode();
	}

	// Voltage scaling range 1 (1.2V) for 16 MHz with 0 wait states (valid up to 24 MHz)
	// VOS2 at 16MHz needs FLASH_LATENCY_1 which may have HAL issues on STM32U0
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	// Enable HSI (16 MHz) and LSI (for RTC)
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;  // No PLL for lower power
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	// Use HSI directly as system clock (16 MHz)
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	// VOS range 1: 0 WS up to 24 MHz
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
#endif
}

// Custom HardFault handler — prints the stacked PC/LR so we can identify the crash site.
// Cortex-M0+ stacks {R0,R1,R2,R3,R12,LR,PC,xPSR} on the MSP before entering the handler.
// The naked wrapper passes SP (= address of that frame) to the C function without disturbing it.
extern "C" void hard_fault_impl(uint32_t *frame)
{
	uint32_t pc  = frame[6];
	uint32_t lr  = frame[5];
	uint32_t r0  = frame[0];
	uint32_t r12 = frame[4];
	Serial.println("\n\n=== HARD FAULT ===");
	Serial.print("PC : 0x"); Serial.println(pc,  HEX);
	Serial.print("LR : 0x"); Serial.println(lr,  HEX);
	Serial.print("R0 : 0x"); Serial.println(r0,  HEX);
	Serial.print("R12: 0x"); Serial.println(r12, HEX);
	Serial.flush();
	while (1) {}
}

extern "C" __attribute__((naked)) void HardFault_Handler(void)
{
	__asm volatile (
		"mov r0, sp                \n"
		"ldr r1, =hard_fault_impl  \n"
		"bx  r1                    \n"
	);
}

void before()
{
	// Enable DBGMCU clock first (STM32U0-specific: RCC->DBGCFGR.DBGEN must be set
	// before any DBGMCU register writes take effect)
	__HAL_RCC_DBGMCU_CLK_ENABLE();
	// Enable debug in STOP mode so SWD stays accessible in sleep
	HAL_DBGMCU_EnableDBGStopMode();

	// hwInit() already called Serial.begin() with LPUART1 (default).
	// Re-initialize Serial with USART2 via ALT1 pins:
	// PA_2_ALT1 = USART2 TX (AF7), PA_3_ALT1 = USART2 RX (AF7)
	Serial.end();  // deinit LPUART1 before switching to USART2
	Serial.setTx(PA_2_ALT1);
	Serial.setRx(PA_3_ALT1);
	Serial.begin(115200);
	delay(100);
	// Print reset cause before clearing flags — critical for diagnosing unexpected resets
	{
		uint32_t csr = RCC->CSR;
		Serial.print("RST=0x"); Serial.print(csr >> 24, HEX);
		if (csr & RCC_CSR_LPWRRSTF)  Serial.print(" LPWR");   // illegal stop/standby entry
		if (csr & RCC_CSR_WWDGRSTF)  Serial.print(" WWDG");
		if (csr & RCC_CSR_IWDGRSTF)  Serial.print(" IWDG");
		if (csr & RCC_CSR_SFTRSTF)   Serial.print(" SFT");    // NVIC_SystemReset()
		if (csr & RCC_CSR_PWRRSTF)   Serial.print(" PWR");    // POR/BOR
		if (csr & RCC_CSR_PINRSTF)   Serial.print(" PIN");    // NRST pin
		if (csr & RCC_CSR_OBLRSTF)   Serial.print(" OBL");
		Serial.println();
		RCC->CSR |= RCC_CSR_RMVF;  // clear all reset flags for next boot
	}

	Serial.println("=== BOOT === (5s upload window)");
	// No Serial.flush() here — it blocks forever if uart_init fails

	// 5-second startup delay: guaranteed window for SWD upload
	// Remove this delay once firmware is confirmed working
	delay(5000);
}

void setup()
{

#ifndef MY_DISABLED_SERIAL
	// Print actual clock speed and power mode info
	// SystemCoreClock = HCLK (CPU bus clock), updated by HAL_RCC_ClockConfig.
	// In MEDIUM mode SYSCLK = 16 MHz HSI but HCLK = 4 MHz (AHB /4), so use
	// SystemCoreClock here instead of HAL_RCC_GetSysClockFreq() (which returns SYSCLK).
	Serial.print("System clock: ");
	Serial.print(SystemCoreClock / 1000000);
	Serial.println(" MHz");
#if POWER_DEBUG_ENABLED
	Serial.print("Run mode: ");
	Serial.println(power_run_mode_name());
	Serial.print("Sleep mode: ");
	Serial.println(power_sleep_mode_name());
#endif
#endif

	// Initialize I2C2 (PB10=SCL, PB11=SDA) at reduced clock for reliability
	Wire.setSCL(I2C2_SCL_PIN);
	Wire.setSDA(I2C2_SDA_PIN);
	Wire.setClock(10000);  // 10 kHz
	Wire.begin();

	// Initialize HDC1080
	hdc1080.begin(0x40);

	// Status LED
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);

	// Configure battery ADC pin as analog input
	// Note: analogRead() is NOT used — STM32duino analog.cpp has a bug on STM32U0:
	// ADC_ChannelConfTypeDef.SamplingTime must be ADC_SAMPLINGTIME_COMMON_1/2 (index),
	// not a raw cycle count (only STM32G0xx is excluded; STM32U0xx should be too).
	// We use direct HAL calls in readADC_U0() instead.
	pinMode(BATTERY_ADC_PIN, INPUT_ANALOG);

	// Load persisted settings from EEPROM (MySensors user state).
	// Uninitialized bytes read as 0xFF; range checks detect first boot.
	{
		uint16_t v;
		v = eeLoad16(EE_INTERVAL_LO);
		if (v >= 10 && v <= 3600) sleepTimeMs = (uint32_t)v * 1000UL;

		v = eeLoad16(EE_BATT_MIN_LO);
		if (v >= 2000 && v <= 4200) battMinMv = v;

		v = eeLoad16(EE_BATT_MAX_LO);
		if (v >= 2000 && v <= 4200) battMaxMv = v;
	}

#ifndef MY_DISABLED_SERIAL
	Serial.print("EEPROM: interval="); Serial.print(sleepTimeMs / 1000);
	Serial.print("s  batt="); Serial.print(battMinMv);
	Serial.print("-"); Serial.print(battMaxMv); Serial.println(" mV");
#endif
}

void presentation()
{
	sendSketchInfo("TempHum Battery Node", "1.1");

	present(CHILD_ID_TEMP, S_TEMP);
	present(CHILD_ID_HUM, S_HUM);
	present(CHILD_ID_BATT, S_MULTIMETER);
	present(CHILD_ID_RSSI,     S_SOUND,   "RSSI dBm");
	present(CHILD_ID_TX_POWER, S_SOUND,   "TX Power dBm");
	present(CHILD_ID_INTERVAL, S_CUSTOM,  "Sleep interval s");
	present(CHILD_ID_BATT_MIN, S_CUSTOM,  "Batt cutoff mV");
	present(CHILD_ID_BATT_MAX, S_CUSTOM,  "Batt full mV");
}

// Called by MySensors when the gateway sends a C_SET message to this node.
// All three adjustable parameters use V_VAR1 payload (integer).
//
//   Sleep interval  : "11;5;1;0;24;<seconds>"  range 10–3600
//   Battery cutoff  : "11;6;1;0;24;<mV>"       range 2000–4200  (e.g. 2850 → 2.85 V)
//   Battery full    : "11;7;1;0;24;<mV>"        range 2000–4200  (e.g. 3600 → 3.60 V)
//
// Values are persisted to EEPROM and take effect immediately.
void receive(const MyMessage &message)
{
	if (message.getType() != V_VAR1) {
		return;
	}

	long raw = message.getLong();
	uint8_t sensor = message.getSensor();

	// Update RAM immediately; EEPROM write is deferred to flushPendingSaves() in
	// loop() to avoid a flash page erase while the RFM69 ISR could fire.
	if (sensor == CHILD_ID_INTERVAL) {
		if (raw < 10)   raw = 10;
		if (raw > 3600) raw = 3600;
		sleepTimeMs = (uint32_t)(uint16_t)raw * 1000UL;
		pendingSave.interval = true;
#ifndef MY_DISABLED_SERIAL
		Serial.print("Interval -> "); Serial.print((uint16_t)raw); Serial.println("s");
#endif

	} else if (sensor == CHILD_ID_BATT_MIN) {
		if (raw < 2000) raw = 2000;
		if (raw > 4200) raw = 4200;
		battMinMv = (uint16_t)raw;
		pendingSave.battMin = true;
#ifndef MY_DISABLED_SERIAL
		Serial.print("Batt cutoff -> "); Serial.print(battMinMv); Serial.println(" mV");
#endif

	} else if (sensor == CHILD_ID_BATT_MAX) {
		if (raw < 2000) raw = 2000;
		if (raw > 4200) raw = 4200;
		battMaxMv = (uint16_t)raw;
		pendingSave.battMax = true;
#ifndef MY_DISABLED_SERIAL
		Serial.print("Batt full   -> "); Serial.print(battMaxMv); Serial.println(" mV");
#endif
	}
}

// Write any parameter changes received via receive() to EEPROM.
// Called from loop() when the radio stack is idle (outside wait()/transport calls),
// so no interrupt can fire during the flash page erase.
static void flushPendingSaves()
{
	if (pendingSave.interval) {
		eeSave16(EE_INTERVAL_LO, (uint16_t)(sleepTimeMs / 1000));
		pendingSave.interval = false;
#ifndef MY_DISABLED_SERIAL
		Serial.print("EEPROM: interval="); Serial.print(sleepTimeMs / 1000); Serial.println("s");
#endif
	}
	if (pendingSave.battMin) {
		eeSave16(EE_BATT_MIN_LO, battMinMv);
		pendingSave.battMin = false;
#ifndef MY_DISABLED_SERIAL
		Serial.print("EEPROM: battMin="); Serial.print(battMinMv); Serial.println(" mV");
#endif
	}
	if (pendingSave.battMax) {
		eeSave16(EE_BATT_MAX_LO, battMaxMv);
		pendingSave.battMax = false;
#ifndef MY_DISABLED_SERIAL
		Serial.print("EEPROM: battMax="); Serial.print(battMaxMv); Serial.println(" mV");
#endif
	}
}

// Direct HAL ADC read for STM32U0, bypassing analogRead().
// Root cause of analogRead() returning 0: STM32duino analog.cpp line ~1125 checks
// only !defined(STM32G0xx) before setting AdcChannelConf.SamplingTime to the raw
// cycle count (e.g. ADC_SAMPLETIME_12CYCLES_5). On STM32U0 the field must be
// ADC_SAMPLINGTIME_COMMON_1 or ADC_SAMPLINGTIME_COMMON_2 (an index into the two
// shared sample-time registers), so HAL_ADC_ConfigChannel() fails and returns 0.
static uint32_t readADC_U0(uint32_t channel)
{
	ADC_HandleTypeDef hadc          = {};
	ADC_ChannelConfTypeDef sConfig  = {};

	__HAL_RCC_ADC_CLK_ENABLE();

	hadc.Instance                   = ADC1;
	hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc.Init.Resolution            = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode          = ADC_SCAN_SEQ_FIXED;
	hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait      = DISABLE;
	hadc.Init.LowPowerAutoPowerOff  = DISABLE;
	hadc.Init.ContinuousConvMode    = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
	hadc.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_160CYCLES_5;  // shared timing slot 1
	hadc.Init.SamplingTimeCommon2   = ADC_SAMPLETIME_160CYCLES_5;  // shared timing slot 2
	hadc.Init.OversamplingMode      = DISABLE;

	HAL_StatusTypeDef r;
	r = HAL_ADC_Init(&hadc);
	if (r != HAL_OK) {
		__HAL_RCC_ADC_CLK_DISABLE();
		return 0;
	}

	// Note: HAL_ADCEx_Calibration_Start() consistently returns HAL_ERROR on STM32U0
	// regardless of ADEN state — skipped. Factory trim (CALFACT) is loaded from OTP
	// at power-on by the STM32U0 ROM; no runtime calibration needed for battery monitoring.

	sConfig.Channel      = channel;
	sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;  // index, not raw cycles

	r = HAL_ADC_ConfigChannel(&hadc, &sConfig);
	if (r != HAL_OK) {
		HAL_ADC_DeInit(&hadc);
		__HAL_RCC_ADC_CLK_DISABLE();
		return 0;
	}

	r = HAL_ADC_Start(&hadc);

	uint32_t value = 0;
	if (r == HAL_OK) {
		r = HAL_ADC_PollForConversion(&hadc, 100);
		if (r == HAL_OK) {
			value = HAL_ADC_GetValue(&hadc);
		}
	}

	HAL_ADC_Stop(&hadc);
	HAL_ADC_DeInit(&hadc);
	__HAL_RCC_ADC_CLK_DISABLE();
	return value;
}

// Measure actual VDDA using VREFINT factory calibration.
// VDDA = VREFINT_CAL_VREF_mV * VREFINT_CAL_VALUE / ADC_reading_of_VREFINT
static float readVDDA()
{
	ADC1_COMMON->CCR |= ADC_CCR_VREFEN;    // enable internal VREFINT path
	HAL_Delay(1);                           // wait ≥12 µs for VREFINT startup
	uint32_t raw = readADC_U0(ADC_CHANNEL_VREFINT);
	ADC1_COMMON->CCR &= ~ADC_CCR_VREFEN;
	if (raw == 0) return 3.0f;             // fallback if ADC failed
	uint16_t cal = *VREFINT_CAL_ADDR;     // factory value measured at VREFINT_CAL_VREF mV
	// ES0602 errata 2.1.2: VREFINT_CAL not programmed on early devices (date codes 333-345,
	// first Nucleo/Discovery batch) — reads 0xFFFF instead of calibration value.
	// Fall back to 3.0 V assumption rather than computing ~115 V VDDA.
	if (cal == 0xFFFF || cal == 0) return 3.0f;
	return ((float)VREFINT_CAL_VREF * (float)cal) / ((float)raw * 1000.0f);
}

float readBatteryVoltage()
{
	// PA0 = ADC1_IN4 (channel 4) on STM32U083, LQFP64 pin 14
	float vdda = readVDDA();
	uint32_t raw = readADC_U0(ADC_CHANNEL_4);
	float voltage = ((float)raw / ADC_MAX_VALUE) * vdda * VOLTAGE_DIVIDER_RATIO;

#ifndef MY_DISABLED_SERIAL
	Serial.print("ADC raw: ");
	Serial.print(raw);
	Serial.print(" -> ");
	Serial.print(voltage, 3);
	Serial.println("V");
#endif

	return voltage;
}

uint8_t voltageToBatteryPercent(float voltage)
{
	float minV = battMinMv / 1000.0f;
	float maxV = battMaxMv / 1000.0f;
	if (voltage <= minV) return 0;
	if (voltage >= maxV) return 100;
	return (uint8_t)(((voltage - minV) / (maxV - minV)) * 100);
}

void loop()
{
	digitalWrite(LED_PIN, HIGH);  // LED on during active state

	// Read HDC1080 sensor
	float temperature = hdc1080.readTemperature();
	float humidity = hdc1080.readHumidity();

	// Read battery voltage
	float batteryVoltage = readBatteryVoltage();
	uint8_t batteryPercent = voltageToBatteryPercent(batteryVoltage);

	// Print to serial
	Serial.print("Temp: ");
	Serial.print(temperature, 1);
	Serial.print("C, Hum: ");
	Serial.print(humidity, 1);
	Serial.print("%, Batt: ");
	Serial.print(batteryVoltage, 2);
	Serial.print("V (");
	Serial.print(batteryPercent);
	Serial.println("%)");

	// Battery empty - send 0% and stop transmitting to preserve remaining capacity.
	// Guard > 2.5V: only trigger on a real dying battery; <2.5V is USB-only / no battery.
	if (batteryVoltage <= (battMinMv / 1000.0f) && batteryVoltage > 2.5f) {
		Serial.println("Battery empty! Sending 0% and shutting down.");
		send(msgBatt.set(batteryVoltage, 2));
		sendBatteryLevel(0);
		digitalWrite(LED_PIN, LOW);
		Serial.flush();
		sleep(0);
		return;
	}

	// Send to gateway
	Serial.println("Sending to gateway...");
	bool ok1 = send(msgTemp.set(temperature, 1));
	bool ok2 = send(msgHum.set(humidity, 1));
	bool ok3 = send(msgBatt.set(batteryVoltage, 2));
	Serial.print("Send results: T=");
	Serial.print(ok1);
	Serial.print(" H=");
	Serial.print(ok2);
	Serial.print(" B=");
	Serial.print(ok3);

	// Send signal report (ATC) - RX RSSI from ACK, TX power level
	int16_t rssi = transportGetSignalReport(SR_RX_RSSI);
	int16_t txPower = transportGetSignalReport(SR_TX_POWER_LEVEL);
	send(msgRSSI.set(rssi));
	send(msgTxPower.set(txPower));
	Serial.print(" RSSI=");
	Serial.print(rssi);
	Serial.print(" TX=");
	Serial.println(txPower);

	// Also send battery level to controller
	sendBatteryLevel(batteryPercent);

	// Report current parameter values only on first boot to initialize the controller.
	// Never resend on wakeups: that would overwrite any value the user changed via the
	// gateway between sleep cycles. Changes arrive via receive() while the node is awake.
	//
	// sendBatteryLevel() is called BEFORE the 60 s wait so it executes while transport
	// state is clean. The MySensors echo mechanism (auto-reply to gateway SETs received
	// during wait()) sends a transport frame after receive() returns, which leaves the
	// transport in a subtly corrupted state — calling sendBatteryLevel() after wait()
	// would hang the node.
	if (firstLoop) {
		send(msgInterval.set((uint16_t)(sleepTimeMs / 1000)));
		send(msgBattMin.set(battMinMv));
		send(msgBattMax.set(battMaxMv));
		firstLoop = false;
		// On first boot, wait 60 s so the user can push custom parameter SETs
		// from the gateway before the node enters its regular sleep cycle.
#ifndef MY_DISABLED_SERIAL
		Serial.println("First boot: waiting 60s for parameter changes...");
		Serial.flush();
#endif
		// Brief 1 Hz blink pattern (6 toggles × 150 ms = 0.9 s) to signal parameter window.
		// Uses delay() — blocks radio for <1 s, acceptable here.
		for (int i = 0; i < 6; i++) {
			digitalWrite(LED_PIN, i & 1 ? LOW : HIGH);
			delay(150);
		}
		digitalWrite(LED_PIN, HIGH);
		// Use 1 s slices instead of a single wait(60000): a gateway SET+echo during
		// a long wait() can leave the RFM69 driver in a state where _process() never
		// returns, hanging wait() indefinitely. Short slices let each call time out
		// cleanly and reset the polling state before the next slice starts.
		for (uint8_t i = 0; i < 60; i++) {
			wait(1000);
		}
	}

	// Wait for any pending SET messages from the controller before sleeping.
	// The controller may retry a failed delivery once it sees the node is active
	// (from the temperature/battery messages above). Without this window, the
	// node sleeps before the retry arrives and the user's change is lost.
	wait(2000);

	// Enter low power sleep (MCU STOP mode + radio sleep)
	digitalWrite(LED_PIN, LOW);  // LED off before sleep
	Serial.print("Sleeping ");
	Serial.print(sleepTimeMs / 1000);
	Serial.println("s...");
	Serial.flush();

	sleep(sleepTimeMs);

	Serial.println("Woke up");
}
