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

// Include power configuration if available
#if __has_include("stm32_power_config.h")
#include "stm32_power_config.h"
#else
// Default power mode configuration
#define POWER_RUN_NORMAL      0
#define POWER_RUN_LOW_POWER   1
#define POWER_SLEEP_SLEEP     0
#define POWER_SLEEP_LP_SLEEP  1
#define POWER_SLEEP_STOP0     2
#define POWER_SLEEP_STOP1     3
#define POWER_SLEEP_STOP2     4
#define POWER_SLEEP_STANDBY   5
#ifndef MY_STM32_RUN_MODE
#define MY_STM32_RUN_MODE     POWER_RUN_NORMAL
#endif
#ifndef MY_STM32_SLEEP_MODE
#define MY_STM32_SLEEP_MODE   POWER_SLEEP_STOP1
#endif
#endif

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
// DIAG: place a marker so we can tell whether the alarm IRQ ever fires.
extern "C" __attribute__((used)) volatile uint32_t alarm_fire_count = 0;

static void alarmCallback(void *data)
{
	(void)data;
	alarmTriggered = true;
	alarm_fire_count++;
	// Drive PB0 (LED) HIGH from the IRQ so we can confirm wake fired even if
	// post-IRQ execution hangs before any Serial output. Reading direct from
	// the BSRR register; PB0 is OUTPUT throughout sleep.
	GPIOB->BSRR = GPIO_PIN_0;
}

// Forward declarations for sleep helper functions
static bool hwSleepInit(void);
static bool hwSleepConfigureAlarm(uint32_t ms);
static void hwSleepRestoreSystemClock(void);
static void hwEnterSleepMode(void);
static void hwPrepareSleep(void);
static void hwRestoreAfterSleep(void);
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

	// Deactivate any stale WUT from a previous firmware run (RTC is battery-backed,
	// survives firmware uploads; a stale armed WUT causes immediate wake from Stop).
	// We use Alarm A for wake (see hwSleepConfigureAlarm), so WUT must be off.
	//
	// ST HAL gap on STM32U0: HAL_RTCEx_DeactivateWakeUpTimer writes RTC_CR without
	// first disabling write protection (WPR), so on this family the WUTE/WUTIE
	// clear is silently dropped — leaving WUT armed across the firmware change.
	// Bracket the call with the WPR unlock/lock sequence and follow it with an
	// explicit SCR write to clear the status flags (SCR is not WPR-protected,
	// but doing it inside the unlocked window costs nothing and keeps the
	// HAL-bug workaround atomic).
	RTC->WPR = 0xCAU;
	RTC->WPR = 0x53U;
	HAL_RTCEx_DeactivateWakeUpTimer(rtc.getHandle());
	WRITE_REG(RTC->SCR, RTC_SCR_CWUTF | RTC_SCR_CALRAF);
	RTC->WPR = 0xFFU;

	// Attach the library's Alarm A interrupt callback. STM32duino's RTC_TAMP_IRQHandler
	// (strong symbol on U0xx) routes Alarm A events to this callback, which clears
	// ALRAF cleanly. Direct WUT use via HAL_RTCEx_SetWakeUpTimer_IT bypasses this
	// path and leaves the wake event unserviced — WFI never returns.
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

	// Compute Alarm A match time = (current RTC time) + sleepSec.
	uint8_t hours   = rtc.getHours();
	uint8_t minutes = rtc.getMinutes();
	uint8_t seconds = rtc.getSeconds();

	uint32_t sleepSec = ms / 1000;
	if (sleepSec == 0) sleepSec = 1;

	uint32_t totalSec = hours * 3600UL + minutes * 60UL + seconds + sleepSec;
	uint8_t alarmHours   = (totalSec / 3600) % 24;
	uint8_t alarmMinutes = (totalSec / 60) % 60;
	uint8_t alarmSeconds = totalSec % 60;

#if !defined(MY_DISABLED_SERIAL)
	Serial.print("Alarm A @ ");
	Serial.print(alarmHours);   Serial.print(":");
	Serial.print(alarmMinutes); Serial.print(":");
	Serial.print(alarmSeconds); Serial.print(" (+");
	Serial.print(sleepSec);     Serial.println("s)");
	Serial.flush();
#endif

	// Configure Alarm A via ST HAL — handles WPR, ALRAE clear/set, sync wait,
	// and AlarmMask correctly. Date and SubSeconds masked so the alarm fires
	// whenever h:m:s match.
	RTC_AlarmTypeDef a = {};
	a.AlarmTime.Hours        = alarmHours;
	a.AlarmTime.Minutes      = alarmMinutes;
	a.AlarmTime.Seconds      = alarmSeconds;
	a.AlarmTime.SubSeconds   = 0;
	a.AlarmTime.TimeFormat   = RTC_HOURFORMAT12_AM;
	a.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	a.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	a.AlarmMask              = RTC_ALARMMASK_DATEWEEKDAY;   // ignore date, compare H/M/S
	a.AlarmSubSecondMask     = RTC_ALARMSUBSECONDMASK_ALL;  // ignore subseconds
	a.AlarmDateWeekDaySel    = RTC_ALARMDATEWEEKDAYSEL_DATE;
	a.AlarmDateWeekDay       = 1;
	a.Alarm                  = RTC_ALARM_A;

	if (HAL_RTC_SetAlarm_IT(rtc.getHandle(), &a, RTC_FORMAT_BIN) != HAL_OK) {
#if !defined(MY_DISABLED_SERIAL)
		Serial.println("HAL_RTC_SetAlarm_IT FAILED");
		Serial.flush();
#endif
		return false;
	}

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
 * @brief Configure unused GPIOs as analog to minimize leakage current
 * @note Pins in use are preserved, all others set to analog mode
 */
static void hwConfigureGpioLowPower(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	// Enable all GPIO clocks for configuration
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// PORTA: Keep PA0 (ADC), PA2/PA3 (USART), PA5/PA6/PA7 (SPI), PA10 (RFM69 IRQ)
	// PA13/PA14 (SWDIO/SWCLK) always kept in AF mode so SWD remains accessible in STOP2
	// Set unused: PA1, PA4, PA8, PA9, PA11, PA12, PA15
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_9 |
	                      GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PORTB: Keep PB6 (RFM69 CS), PB10/PB11 (I2C2), PB0 (wake indicator, driven LOW before sleep)
	// Set unused: PB1, PB2, PB3, PB4, PB5, PB7, PB8, PB9, PB12-PB15
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
	                      GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 |
	                      GPIO_PIN_9 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PORTC: All unused (PC13 is often user button, PC14/PC15 are LSE if used)
	GPIO_InitStruct.Pin = GPIO_PIN_ALL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// PORTD: All unused
	GPIO_InitStruct.Pin = GPIO_PIN_ALL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	// Disable GPIO clocks for unused ports
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
}

/**
 * @brief Prepare peripherals for sleep (disable clocks)
 */
static void hwPrepareSleep(void)
{
#if defined(STM32U0xx)
	// STM32U0: DBGMCU clock must be enabled via RCC->DBGCFGR.DBGEN before
	// DBGMCU register writes take effect (unlike most other STM32 families).
	// FORCE DBG_STOP=0 regardless of MY_DEBUG: enabling DBG in Stop keeps clocks
	// running and may interfere with WUT wakeup on STM32U0. SWD upload still works
	// through the 5-second boot window in before() or via Under-Reset upload.
	__HAL_RCC_DBGMCU_CLK_ENABLE();
	DBGMCU->CR = 0;
	__HAL_RCC_DBGMCU_CLK_DISABLE();

	// Arm EXTI28 (all RTC events on STM32U0) and internal-wake belt-and-suspenders.
	// EXTI28 is a "direct" line — IMR28=1 is sufficient; RTSR/FTSR have no effect.
	// NVIC IRQ 2 (RTC_TAMP_IRQn) is the actual wake vector; we enable it explicitly
	// in case STM32RTC library hasn't already done so.
	// Clear any pending RTC event flags BEFORE arming EXTI28 / enabling NVIC IRQ.
	// EXTI28 is a direct line that follows the RTC event flags (WUTF | ALRAF | …).
	// If any flag is stale (e.g. WUTF left over from a previous firmware's WUT),
	// the IRQ fires the instant it is enabled and refires endlessly because the
	// STM32duino RTC handler only services the events for which a callback was
	// attached (Alarm A in our case).
	WRITE_REG(RTC->SCR, RTC_SCR_CWUTF | RTC_SCR_CALRAF);

	EXTI->IMR1 |= (1UL << 28);
	PWR->CR3   |= PWR_CR3_EIWUL;
	NVIC_ClearPendingIRQ(RTC_TAMP_IRQn);
	NVIC_EnableIRQ(RTC_TAMP_IRQn);

#if !defined(MY_DISABLED_SERIAL)
	// Print BEFORE hwConfigureGpioLowPower sets PA2 (UART TX) to analog — any
	// Serial.print after that is silently lost.
	// RTC_CR bits: bit8=ALRAE, bit10=WUTE, bit12=ALRAIE, bit14=WUTIE
	// RTC_SR bits: bit0=ALRAF, bit2=WUTF
	Serial.print("PRE CR=0x");  Serial.print(RTC->CR, HEX);
	Serial.print(" SR=0x");     Serial.print(RTC->SR, HEX);
	Serial.print(" WUTR=");     Serial.print(RTC->WUTR);
	Serial.print(" ICSR=0x");   Serial.print(RTC->ICSR, HEX);
	Serial.print(" ALRMAR=0x"); Serial.print(RTC->ALRMAR, HEX);
	Serial.print(" ALRMASSR=0x"); Serial.print(RTC->ALRMASSR, HEX);
	Serial.print(" PREDIV=0x"); Serial.print(RTC->PRER, HEX);
	Serial.print(" CR3=0x");    Serial.print(PWR->CR3, HEX);
	Serial.print(" IMR28=");    Serial.print((EXTI->IMR1 >> 28) & 1);
	Serial.print(" NVIC2=");    Serial.print(NVIC_GetEnableIRQ(RTC_TAMP_IRQn));
	Serial.print(" t=");        Serial.print(rtc.getHours());
	Serial.print(":");          Serial.print(rtc.getMinutes());
	Serial.print(":");          Serial.println(rtc.getSeconds());
	Serial.flush();
#endif
#endif

	// GPIOs to analog and peripheral clocks off — DO THIS LAST (kills UART TX).
	hwConfigureGpioLowPower();
	__HAL_RCC_SPI1_CLK_DISABLE();
	__HAL_RCC_I2C1_CLK_DISABLE();
	__HAL_RCC_I2C2_CLK_DISABLE();
	__HAL_RCC_ADC_CLK_DISABLE();
}

/**
 * @brief Restore peripherals after wake-up
 */
static void hwRestoreAfterSleep(void)
{
	// Re-enable GPIO clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// Re-enable peripheral clocks
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_ADC_CLK_ENABLE();

	// Reinitialize SPI1 pins (PA5=SCK, PA6=MISO, PA7=MOSI)
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Reinitialize PB0 (wake indicator) as output LOW — app code drives it HIGH after return
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // keep LOW until app drives HIGH

	// Reinitialize RFM69 CS pin (PB6)
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // CS high (inactive)

	// Reinitialize RFM69 IRQ pin (PA10)
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Reinitialize I2C2 pins (PB10=SCL, PB11=SDA)
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Reinitialize ADC pin (PA0)
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Enter the configured sleep mode
 * @note Sleep mode is selected at compile time via MY_STM32_SLEEP_MODE
 */
static void hwEnterSleepMode(void)
{
#if MY_STM32_SLEEP_MODE == POWER_SLEEP_SLEEP
	// ==================== Sleep Mode ====================
	// CPU halted, peripherals running, fast wake-up
	// Current: ~1 mA (varies with peripherals enabled)
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	HAL_ResumeTick();

#elif MY_STM32_SLEEP_MODE == POWER_SLEEP_LP_SLEEP
	// ==================== Low Power Sleep Mode ====================
	// Requires Low Power Run mode (2 MHz MSI)
	// CPU halted, LP regulator, limited peripherals
	// Current: ~100-200 µA
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	HAL_ResumeTick();

#elif MY_STM32_SLEEP_MODE == POWER_SLEEP_STOP0
	// ==================== Stop 0 Mode ====================
	// Fast wake-up, SRAM retained
	// Current: ~10-20 µA
	HAL_SuspendTick();
#if defined(STM32U0xx)
	HAL_PWREx_EnterSTOP0Mode(PWR_STOPENTRY_WFI);
#else
	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
#endif
	// Resume tick BEFORE SystemClock_Config (HAL_RCC_*Config use HAL_GetTick
	// for their internal timeouts; with the tick frozen they spin forever).
	HAL_ResumeTick();
	hwSleepRestoreSystemClock();

#elif MY_STM32_SLEEP_MODE == POWER_SLEEP_STOP1
	// ==================== Stop 0 Mode (manual — U0 HAL lacks wrapper) ==========
	// LPMS=0b000 = Stop 0: main regulator stays ON, least disruption to peripherals.
	// STM32U0 HAL only provides HAL_PWREx_EnterSTOP1Mode / EnterSTOP2Mode, so Stop 0
	// must be entered by hand. If Stop 1's LP-regulator transition is what blocks
	// WUT wake, Stop 0 should wake reliably.
	HAL_SuspendTick();
#if defined(STM32U0xx)
	GPIOB->BRR = GPIO_PIN_0;  // LED OFF — goes back ON after WFI if we wake
	MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, 0);            // LPMS=000 (Stop 0)
	SET_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
	__WFI();
	CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);
	GPIOB->BSRR = GPIO_PIN_0;  // LED ON immediately after WFI returns
#else
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
#endif
	// Resume tick BEFORE SystemClock_Config (see STOP0 comment).
	HAL_ResumeTick();
	hwSleepRestoreSystemClock();

#elif MY_STM32_SLEEP_MODE == POWER_SLEEP_STOP2
	// ==================== Stop 2 Mode ====================
	// Lowest Stop power, limited peripherals
	// Current: ~1-3 µA
	HAL_SuspendTick();
#if defined(STM32U0xx)
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
#else
	// Fallback to Stop 1 if Stop 2 not available
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
#endif
	// Resume tick BEFORE SystemClock_Config (see STOP0 comment).
	HAL_ResumeTick();
	hwSleepRestoreSystemClock();

#elif MY_STM32_SLEEP_MODE == POWER_SLEEP_STANDBY
	// ==================== Standby Mode ====================
	// Lowest power, RAM lost, only RTC/IWDG preserved
	// Current: ~300 nA
	// WARNING: System will reset on wake-up!
#if defined(STM32U0xx)
	// Enable WKUP pin if needed (e.g., PA0)
	// HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_PWR_EnterSTANDBYMode();
#else
	HAL_PWR_EnterSTANDBYMode();
#endif
	// Will not reach here - system resets on wake

#else
	// Default fallback to Stop 1
	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	hwSleepRestoreSystemClock();
	HAL_ResumeTick();
#endif
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

	// Sleep/LP Sleep modes don't need RTC for timed wake-up with short durations
	// but we use RTC alarm for consistent behavior across all modes
#if MY_STM32_SLEEP_MODE != POWER_SLEEP_STANDBY
	// Initialize RTC if needed (not needed for Standby with WKUP pin only)
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
#endif

	// Reset sleep remaining
	sleepRemainingMs = 0ul;

	// Clear any pending wake-up flags
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

#if !defined(MY_DISABLED_SERIAL)
	Serial.flush();
	delay(10);
#endif

	// Prepare peripherals for sleep
	hwPrepareSleep();

	// Enter the configured sleep mode
	hwEnterSleepMode();

	// ========== WOKE UP ==========

	// Restore peripherals
	hwRestoreAfterSleep();

#if !defined(MY_DISABLED_SERIAL)
	Serial.print("POST SR=0x"); Serial.print(RTC->SR, HEX);
	Serial.print(" alrm=");     Serial.print(alarmTriggered);
	Serial.print(" t=");        Serial.print(rtc.getHours());
	Serial.print(":");          Serial.print(rtc.getMinutes());
	Serial.print(":");          Serial.println(rtc.getSeconds());
	Serial.flush();
#endif

#if MY_STM32_SLEEP_MODE != POWER_SLEEP_STANDBY
	// Disable WUT and clear its flag so it doesn't re-arm before next sleep.
	HAL_RTCEx_DeactivateWakeUpTimer(rtc.getHandle());
	WRITE_REG(RTC->SCR, RTC_SCR_CWUTF);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
#endif

	return MY_WAKE_UP_BY_TIMER;
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

#if MY_STM32_SLEEP_MODE != POWER_SLEEP_STANDBY
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
#endif

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

	// Prepare peripherals for sleep
	hwPrepareSleep();

	// Enter the configured sleep mode
	hwEnterSleepMode();

	// ========== WOKE UP ==========

	// Restore peripherals
	hwRestoreAfterSleep();

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

#if MY_STM32_SLEEP_MODE != POWER_SLEEP_STANDBY
	// Disable WUT and clear its flag
	HAL_RTCEx_DeactivateWakeUpTimer(rtc.getHandle());
	WRITE_REG(RTC->SCR, RTC_SCR_CWUTF);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
#endif

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
