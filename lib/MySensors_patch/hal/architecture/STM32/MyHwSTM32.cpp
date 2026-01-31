/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2026 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

/**
 * @file MyHwSTM32.cpp
 * @brief Hardware abstraction layer for STM32 microcontrollers using STM32duino core
 *
 * This implementation uses the official STM32duino Arduino core which provides
 * STM32Cube HAL underneath. It supports a wide range of STM32 families including
 * F0, F1, F4, L0, L4, G0, G4, H7, and more.
 *
 * STM32U0 Note: The RTC wake-up timer on STM32U0 series does not work reliably.
 * This implementation uses RTC Alarm A via the STM32RTC library instead, which
 * provides reliable timed wake-up from STOP mode.
 *
 * Tested on:
 * - STM32F401CC/CE Black Pill
 * - STM32F411CE Black Pill
 * - STM32U083RC
 *
 * Pin Mapping Example (STM32F4 Black Pill):
 *
 * nRF24L01+ Radio (SPI1):
 * - SCK:  PA5
 * - MISO: PA6
 * - MOSI: PA7
 * - CSN:  PA4
 * - CE:   PB0 (configurable via MY_RF24_CE_PIN)
 *
 * RFM69/RFM95 Radio (SPI1):
 * - SCK:  PA5
 * - MISO: PA6
 * - MOSI: PA7
 * - CS:   PA4
 * - IRQ:  PA3 (configurable)
 * - RST:  PA2 (configurable)
 */

#include "MyHwSTM32.h"
#include <STM32RTC.h>

// Sleep mode state variables
static volatile uint8_t _wokeUpByInterrupt = INVALID_INTERRUPT_NUM;
static volatile uint8_t _wakeUp1Interrupt = INVALID_INTERRUPT_NUM;
static volatile uint8_t _wakeUp2Interrupt = INVALID_INTERRUPT_NUM;
static uint32_t sleepRemainingMs = 0ul;

// Use STM32RTC library for reliable alarm-based wake-up
static STM32RTC &rtc = STM32RTC::getInstance();
static volatile bool alarmTriggered = false;
static bool rtcInitialized = false;

// Callback for RTC alarm
static void alarmCallback(void *data)
{
	(void)data;
	alarmTriggered = true;
}

// Forward declarations for sleep helper functions
static bool hwSleepInit(void);
static bool hwSleepConfigureAlarm(uint32_t ms);
static void hwSleepRestoreSystemClock(void);
static void wakeUp1ISR(void);
static void wakeUp2ISR(void);

bool hwInit(void)
{
#if !defined(MY_DISABLED_SERIAL)
	MY_SERIALDEVICE.begin(MY_BAUD_RATE);
#if defined(MY_GATEWAY_SERIAL)
	// Wait for serial port to connect (needed for native USB)
	while (!MY_SERIALDEVICE) {
		; // Wait for serial port connection
	}
#endif
#endif

	// STM32duino EEPROM library auto-initializes on first use
	// No explicit initialization required
	return true;
}

void hwReadConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *dst = static_cast<uint8_t *>(buf);
	int pos = reinterpret_cast<int>(addr);

	for (size_t i = 0; i < length; i++) {
		dst[i] = EEPROM.read(pos + i);
	}
}

void hwWriteConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *src = static_cast<uint8_t *>(buf);
	int pos = reinterpret_cast<int>(addr);

	for (size_t i = 0; i < length; i++) {
		EEPROM.update(pos + i, src[i]);
	}
}

uint8_t hwReadConfig(const int addr)
{
	return EEPROM.read(addr);
}

void hwWriteConfig(const int addr, uint8_t value)
{
	EEPROM.update(addr, value);
}

void hwWatchdogReset(void)
{
#if defined(HAL_IWDG_MODULE_ENABLED) && defined(IWDG)
	// Reset independent watchdog if enabled
	// Use direct register write to reload watchdog counter
	// This works whether IWDG was initialized by HAL or LL drivers
	IWDG->KR = IWDG_KEY_RELOAD;
#endif
}

void hwReboot(void)
{
	NVIC_SystemReset();
}

void hwRandomNumberInit(void)
{
	// Use internal temperature sensor and ADC noise as entropy source
	// This provides reasonably good random seed values

#ifdef ADC1
	uint32_t seed = 0;

	// Read multiple samples from different sources for entropy
	for (uint8_t i = 0; i < 32; i++) {
		uint32_t value = 0;

#ifdef TEMP_SENSOR_AVAILABLE
		// Try to read internal temperature sensor if available
		value ^= analogRead(ATEMP);
#endif

#ifdef VREF_AVAILABLE
		// Mix in internal voltage reference reading
		value ^= analogRead(AVREF);
#endif

		// Mix in current time
		value ^= hwMillis();

		// Mix in system tick
		value ^= micros();

		// Accumulate into seed
		seed ^= (value & 0x7) << (i % 29);

		// Small delay to ensure values change
		delayMicroseconds(100);
	}

	randomSeed(seed);
#else
	// Fallback: use millis as weak entropy source
	randomSeed(hwMillis());
#endif // ADC1
}

bool hwUniqueID(unique_id_t *uniqueID)
{
#ifdef UID_BASE
	// STM32 unique device ID is stored at a fixed address
	// Length is 96 bits (12 bytes) but we store 16 bytes for compatibility
	(void)memcpy((uint8_t *)uniqueID, (uint32_t *)UID_BASE, 12);
	(void)memset(static_cast<void *>(uniqueID + 12), MY_HWID_PADDING_BYTE, 4); // padding
	return true;
#else
	// Unique ID not available on this variant
	return false;
#endif
}

uint16_t hwCPUVoltage(void)
{
#if defined(VREF_AVAILABLE) && defined(AVREF) && defined(__HAL_RCC_ADC1_CLK_ENABLE)
	// Read internal voltage reference to calculate VDD
	// VREFINT is typically 1.2V (varies by STM32 family)

	uint32_t vrefint = analogRead(AVREF);

	if (vrefint > 0) {
		// Calculate VDD in millivolts
		// Formula: VDD = 3.3V * 4096 / ADC_reading
		// Adjusted: VDD = 1200mV * 4096 / vrefint_reading
		return (uint16_t)((1200UL * 4096UL) / vrefint);
	}
#endif

	// Return typical 3.3V if measurement not available
	return 3300;
}

uint16_t hwCPUFrequency(void)
{
	// Return CPU frequency in 0.1 MHz units
	// F_CPU is defined by the build system (e.g., 84000000 for 84 MHz)
	return F_CPU / 100000UL;
}

int8_t hwCPUTemperature(void)
{
#if defined(TEMP_SENSOR_AVAILABLE) && defined(ATEMP) && defined(__HAL_RCC_ADC1_CLK_ENABLE)
	// Read internal temperature sensor
	// Note: Requires calibration values for accurate results

	int32_t temp_raw = analogRead(ATEMP);

#ifdef TEMP110_CAL_ADDR
	// Use factory calibration if available (STM32F4, L4, etc.)
	uint16_t *temp30_cal = (uint16_t *)TEMP30_CAL_ADDR;
	uint16_t *temp110_cal = (uint16_t *)TEMP110_CAL_ADDR;

	if (temp30_cal && temp110_cal && *temp110_cal != *temp30_cal) {
		// Calculate temperature using two-point calibration
		// Formula: T = ((110-30) / (CAL_110 - CAL_30)) * (raw - CAL_30) + 30
		int32_t temp = 30 + ((110 - 30) * (temp_raw - *temp30_cal)) /
		               (*temp110_cal - *temp30_cal);

		// Apply user calibration
		temp = (temp - MY_STM32_TEMPERATURE_OFFSET) / MY_STM32_TEMPERATURE_GAIN;

		return (int8_t)temp;
	}
#endif // TEMP110_CAL_ADDR

	// Fallback: use typical values (less accurate)
	// Typical slope: 2.5 mV/°C, V25 = 0.76V for STM32F4
	// This is a rough approximation
	float voltage = (temp_raw * 3.3f) / 4096.0f;
	int32_t temp = 25 + (int32_t)((voltage - 0.76f) / 0.0025f);

	return (int8_t)((temp - MY_STM32_TEMPERATURE_OFFSET) / MY_STM32_TEMPERATURE_GAIN);
#else
	// Temperature sensor not available
	return FUNCTION_NOT_SUPPORTED;
#endif
}

uint16_t hwFreeMem(void)
{
	// Calculate free heap memory
	// This uses newlib's mallinfo if available

#ifdef STACK_TOP
	extern char *__brkval;
	extern char __heap_start;

	char *heap_end = __brkval ? __brkval : &__heap_start;
	char stack_var;

	// Calculate space between heap and stack
	return (uint16_t)(&stack_var - heap_end);
#else
	// Alternative method: try to allocate and measure
	// Not implemented to avoid fragmentation
	return FUNCTION_NOT_SUPPORTED;
#endif
}

// ======================== Sleep Mode Helper Functions ========================

/**
 * @brief Initialize RTC using STM32RTC library for reliable alarm-based wake-up
 * @return true if successful, false on error
 *
 * Note: The RTC wake-up timer on STM32U0 series does not work reliably.
 * This implementation uses RTC Alarm A instead, which provides reliable
 * timed wake-up from STOP mode.
 */
static bool hwSleepInit(void)
{
	if (rtcInitialized) {
		return true;
	}

	// Use STM32RTC library which handles all the low-level RTC configuration
	rtc.setClockSource(STM32RTC::LSI_CLOCK);
	rtc.begin();

	// Attach alarm callback for wake-up
	rtc.attachInterrupt(alarmCallback, nullptr, STM32RTC::ALARM_A);

	rtcInitialized = true;
	return true;
}

/**
 * @brief Configure RTC alarm for specified duration using STM32RTC library
 * @param ms Milliseconds to sleep (0 = disable alarm)
 * @return true if successful, false on error
 */
static bool hwSleepConfigureAlarm(uint32_t ms)
{
	if (!rtcInitialized) {
		if (!hwSleepInit()) {
			return false;
		}
	}

	if (ms == 0) {
		rtc.disableAlarm(STM32RTC::ALARM_A);
		return true;
	}

	// Get current time and add sleep duration
	uint8_t hours = rtc.getHours();
	uint8_t minutes = rtc.getMinutes();
	uint8_t seconds = rtc.getSeconds();

	// Convert ms to seconds
	uint32_t sleepSec = ms / 1000;
	if (sleepSec == 0) {
		sleepSec = 1;
	}

	// Calculate alarm time
	uint32_t totalSec = hours * 3600UL + minutes * 60UL + seconds + sleepSec;
	uint8_t alarmHours = (totalSec / 3600) % 24;
	uint8_t alarmMinutes = (totalSec / 60) % 60;
	uint8_t alarmSeconds = totalSec % 60;

#if !defined(MY_DISABLED_SERIAL)
	Serial.print("WUT: ");
	Serial.print(sleepSec);
	Serial.println("s");
	Serial.flush();
#endif

	// Set alarm to match hours:minutes:seconds
	rtc.setAlarmTime(alarmHours, alarmMinutes, alarmSeconds, STM32RTC::ALARM_A);
	rtc.enableAlarm(rtc.MATCH_HHMMSS, STM32RTC::ALARM_A);

	alarmTriggered = false;
	return true;
}

/**
 * @brief Restore system clock after wake-up from STOP mode
 */
static void hwSleepRestoreSystemClock(void)
{
	SystemClock_Config();
}

/**
 * @brief ISR for wake-up interrupt 1
 */
static void wakeUp1ISR(void)
{
	_wokeUpByInterrupt = _wakeUp1Interrupt;
}

/**
 * @brief ISR for wake-up interrupt 2
 */
static void wakeUp2ISR(void)
{
	_wokeUpByInterrupt = _wakeUp2Interrupt;
}

// ======================== Public Sleep Functions ========================

uint32_t hwGetSleepRemaining(void)
{
	return sleepRemainingMs;
}

int8_t hwSleep(uint32_t ms)
{
#if !defined(MY_DISABLED_SERIAL)
	Serial.print("Sleep ");
	Serial.print(ms / 1000);
	Serial.println("s");
	Serial.flush();
#endif

	// Initialize RTC if needed
	if (!rtcInitialized) {
		if (!hwSleepInit()) {
#if !defined(MY_DISABLED_SERIAL)
			Serial.println("RTC init FAILED");
#endif
			return MY_SLEEP_NOT_POSSIBLE;
		}
	}

	// Configure RTC alarm for wake-up
	if (ms > 0) {
		if (!hwSleepConfigureAlarm(ms)) {
#if !defined(MY_DISABLED_SERIAL)
			Serial.println("Alarm config FAILED");
#endif
			return MY_SLEEP_NOT_POSSIBLE;
		}
	}

	// Reset sleep remaining
	sleepRemainingMs = 0ul;

	// Clear any pending wake-up flags
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

#if !defined(MY_DISABLED_SERIAL)
	Serial.flush();
	delay(10);
#endif

	// Disable peripheral clocks for lower power
	__HAL_RCC_SPI1_CLK_DISABLE();
	__HAL_RCC_I2C1_CLK_DISABLE();
	__HAL_RCC_I2C2_CLK_DISABLE();
	__HAL_RCC_ADC_CLK_DISABLE();

#if defined(STM32U0xx)
	// Disable debug during sleep (DBGMCU keeps clocks running)
	DBGMCU->CR = 0;

	// Enable EXTI line 17 for RTC Alarm wake-up
	// On STM32U0, RTC Alarm is on EXTI line 17 (rising edge)
	EXTI->IMR1 |= (1UL << 17);   // Interrupt mask
	EXTI->RTSR1 |= (1UL << 17);  // Rising edge trigger
	EXTI->RPR1 = (1UL << 17);    // Clear pending

	// Enable Internal Wake-up Line for RTC
	PWR->CR3 |= PWR_CR3_EIWUL;
#endif

	// Suspend SysTick before entering STOP
	HAL_SuspendTick();

	// Enter STOP mode (low-power regulator, wake on interrupt)
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	// ========== WOKE UP ==========

	// Restore system clock (STOP mode reverts to HSI)
	hwSleepRestoreSystemClock();

	// Resume SysTick
	HAL_ResumeTick();

	// Re-enable peripheral clocks
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_ADC_CLK_ENABLE();

#if !defined(MY_DISABLED_SERIAL)
	Serial.println("Woke");
	Serial.flush();
#endif

	// Disable alarm after wake-up
	rtc.disableAlarm(STM32RTC::ALARM_A);

	return alarmTriggered ? MY_WAKE_UP_BY_TIMER : MY_WAKE_UP_BY_TIMER;
}

int8_t hwSleep(const uint8_t interrupt, const uint8_t mode, uint32_t ms)
{
	// Delegate to dual-interrupt variant with INVALID second interrupt
	return hwSleep(interrupt, mode, INVALID_INTERRUPT_NUM, 0, ms);
}

int8_t hwSleep(const uint8_t interrupt1, const uint8_t mode1,
               const uint8_t interrupt2, const uint8_t mode2, uint32_t ms)
{
#if !defined(MY_DISABLED_SERIAL)
	Serial.print("Sleep ");
	Serial.print(ms / 1000);
	Serial.print("s i1=");
	Serial.print(interrupt1);
	Serial.print(" i2=");
	Serial.println(interrupt2);
	Serial.flush();
#endif

	// Initialize RTC if needed
	if (!rtcInitialized) {
		if (!hwSleepInit()) {
#if !defined(MY_DISABLED_SERIAL)
			Serial.println("RTC init FAILED");
#endif
			return MY_SLEEP_NOT_POSSIBLE;
		}
	}

	// Configure RTC alarm for wake-up (if ms > 0)
	if (ms > 0) {
		if (!hwSleepConfigureAlarm(ms)) {
#if !defined(MY_DISABLED_SERIAL)
			Serial.println("Alarm config FAILED");
#endif
			return MY_SLEEP_NOT_POSSIBLE;
		}
	}

	// Reset sleep remaining
	sleepRemainingMs = 0ul;

	// Configure interrupt wake-up sources
	_wakeUp1Interrupt = interrupt1;
	_wakeUp2Interrupt = interrupt2;
	_wokeUpByInterrupt = INVALID_INTERRUPT_NUM;

	// Attach interrupts
	MY_CRITICAL_SECTION {
		if (interrupt1 != INVALID_INTERRUPT_NUM) {
			attachInterrupt(interrupt1, wakeUp1ISR, mode1);
		}
		if (interrupt2 != INVALID_INTERRUPT_NUM) {
			attachInterrupt(interrupt2, wakeUp2ISR, mode2);
		}
	}

	// Clear wake-up flags
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

#if !defined(MY_DISABLED_SERIAL)
	Serial.flush();
	delay(10);
#endif

	// Disable peripheral clocks
	__HAL_RCC_SPI1_CLK_DISABLE();
	__HAL_RCC_I2C1_CLK_DISABLE();
	__HAL_RCC_I2C2_CLK_DISABLE();
	__HAL_RCC_ADC_CLK_DISABLE();

#if defined(STM32U0xx)
	DBGMCU->CR = 0;

	// Enable EXTI line 17 for RTC Alarm wake-up
	EXTI->IMR1 |= (1UL << 17);
	EXTI->RTSR1 |= (1UL << 17);
	EXTI->RPR1 = (1UL << 17);
	PWR->CR3 |= PWR_CR3_EIWUL;
#endif

	// Suspend SysTick
	HAL_SuspendTick();

	// Enter STOP mode
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	// After wake-up: restore system clock
	hwSleepRestoreSystemClock();

	// Resume SysTick
	HAL_ResumeTick();

	// Re-enable peripheral clocks
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_ADC_CLK_ENABLE();

#if !defined(MY_DISABLED_SERIAL)
	Serial.println("Woke");
	Serial.flush();
#endif

	// Detach interrupts
	if (interrupt1 != INVALID_INTERRUPT_NUM) {
		detachInterrupt(interrupt1);
	}
	if (interrupt2 != INVALID_INTERRUPT_NUM) {
		detachInterrupt(interrupt2);
	}

	// Disable alarm
	rtc.disableAlarm(STM32RTC::ALARM_A);

	// Determine wake-up source
	int8_t ret = MY_WAKE_UP_BY_TIMER;
	if (_wokeUpByInterrupt != INVALID_INTERRUPT_NUM) {
		ret = (int8_t)_wokeUpByInterrupt;
	}

	// Reset interrupt tracking
	_wokeUpByInterrupt = INVALID_INTERRUPT_NUM;
	_wakeUp1Interrupt = INVALID_INTERRUPT_NUM;
	_wakeUp2Interrupt = INVALID_INTERRUPT_NUM;

	return ret;
}
