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
// Run modes: POWER_RUN_NORMAL (16 MHz), POWER_RUN_LOW_POWER (2 MHz)
// Sleep modes: POWER_SLEEP_SLEEP, POWER_SLEEP_LP_SLEEP, POWER_SLEEP_STOP0,
//              POWER_SLEEP_STOP1, POWER_SLEEP_STOP2, POWER_SLEEP_STANDBY
#define MY_STM32_RUN_MODE    POWER_RUN_NORMAL   // Test: 16 MHz HSI to verify ADC
#define MY_STM32_SLEEP_MODE  POWER_SLEEP_STOP2     // Stop 2 mode (~1-3 µA)
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
#define MY_NODE_ID 11
#define MY_RFM69_SPI_SPEED (1*1000000ul)

#include <MySensors.h>
#include <Wire.h>
#include "ClosedCube_HDC1080.h"

// I2C2 pins for HDC1080
#define I2C2_SDA_PIN PB11
#define I2C2_SCL_PIN PB10

// Sleep interval between reports (in milliseconds)
#define SLEEP_TIME 60000  // 60 seconds

// Child IDs
#define CHILD_ID_TEMP 0
#define CHILD_ID_HUM 1
#define CHILD_ID_BATT 2
#define CHILD_ID_RSSI 3
#define CHILD_ID_TX_POWER 4

// Battery voltage ADC pin
#define BATTERY_ADC_PIN PA0  // ADC1_IN4, LQFP64 pin 14
// Voltage divider ratio: 2:1 (100k/100k), battery=3.2V → PA0=1.6V
#define VOLTAGE_DIVIDER_RATIO 2.0
// ADC resolution (12-bit)
#define ADC_MAX_VALUE 4095
// Fixed VDDA: MCU/ADC powered by 2.8V LDO (regulated, no need to measure)
#define VDD_LDO_VOLTAGE 2.8f

// HDC1080 sensor
ClosedCube_HDC1080 hdc1080;

// MySensors messages
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgBatt(CHILD_ID_BATT, V_VOLTAGE);
MyMessage msgRSSI(CHILD_ID_RSSI, V_LEVEL);
MyMessage msgTxPower(CHILD_ID_TX_POWER, V_LEVEL);

// Custom system clock configuration with power mode support
// Supports: Normal Run (16 MHz HSI) or Low Power Run (2 MHz MSI)
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

#else
	// ==================== Normal Run Mode (16 MHz HSI) ====================
	// Exit Low Power Run mode if we're in it (after STOP wake-up)
	if (READ_BIT(PWR->SR2, PWR_SR2_REGLPF)) {
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

void before()
{
	// Enable debug in STOP mode early so SWD stays accessible in sleep
	HAL_DBGMCU_EnableDBGStopMode();

	// hwInit() already called Serial.begin() with LPUART1 (default).
	// Re-initialize Serial with USART2 via ALT1 pins:
	// PA_2_ALT1 = USART2 TX (AF7), PA_3_ALT1 = USART2 RX (AF7)
	Serial.end();  // deinit LPUART1 before switching to USART2
	Serial.setTx(PA_2_ALT1);
	Serial.setRx(PA_3_ALT1);
	Serial.begin(115200);
	delay(100);
	Serial.println("\n\n=== BOOT === (5s upload window)");
	// No Serial.flush() here — it blocks forever if uart_init fails

	// 5-second startup delay: guaranteed window for SWD upload
	// Remove this delay once firmware is confirmed working
	delay(5000);
}

void setup()
{

#ifndef MY_DISABLED_SERIAL
	// Print actual clock speed and power mode info
	Serial.print("System clock: ");
	Serial.print(HAL_RCC_GetSysClockFreq() / 1000000);
	Serial.println(" MHz");
#if POWER_DEBUG_ENABLED
	Serial.print("Run mode: ");
	Serial.println(power_run_mode_name());
	Serial.print("Sleep mode: ");
	Serial.println(power_sleep_mode_name());
#endif
#endif

	// Initialize I2C2 (PB10=SCL, PB11=SDA)
	Wire.setSCL(I2C2_SCL_PIN);
	Wire.setSDA(I2C2_SDA_PIN);
	Wire.begin();

	// Initialize HDC1080
	hdc1080.begin(0x40);

	// Configure battery ADC pin as analog input
	// Note: analogRead() is NOT used — STM32duino analog.cpp has a bug on STM32U0:
	// ADC_ChannelConfTypeDef.SamplingTime must be ADC_SAMPLINGTIME_COMMON_1/2 (index),
	// not a raw cycle count (only STM32G0xx is excluded; STM32U0xx should be too).
	// We use direct HAL calls in readADC_U0() instead.
	pinMode(BATTERY_ADC_PIN, INPUT_ANALOG);

#ifndef MY_DISABLED_SERIAL
	Serial.println("HDC1080 + Battery sensor ready");
#endif
}

void presentation()
{
	sendSketchInfo("TempHum Battery Node", "1.0");

	present(CHILD_ID_TEMP, S_TEMP);
	present(CHILD_ID_HUM, S_HUM);
	present(CHILD_ID_BATT, S_MULTIMETER);
	present(CHILD_ID_RSSI, S_SOUND, "RSSI dBm");
	present(CHILD_ID_TX_POWER, S_SOUND, "TX Power dBm");
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

	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		__HAL_RCC_ADC_CLK_DISABLE();
		return 0;
	}

	sConfig.Channel      = channel;
	sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;  // index, not raw cycles

	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		HAL_ADC_DeInit(&hadc);
		__HAL_RCC_ADC_CLK_DISABLE();
		return 0;
	}

	HAL_ADCEx_Calibration_Start(&hadc);

	uint32_t value = 0;
	if (HAL_ADC_Start(&hadc) == HAL_OK &&
	    HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
		value = HAL_ADC_GetValue(&hadc);
	}

	HAL_ADC_Stop(&hadc);
	HAL_ADC_DeInit(&hadc);
	__HAL_RCC_ADC_CLK_DISABLE();
	return value;
}

float readBatteryVoltage()
{
	// PA0 = ADC1_IN4 (channel 4) on STM32U083, LQFP64 pin 14
	uint32_t raw = readADC_U0(ADC_CHANNEL_4);
	float voltage = ((float)raw / ADC_MAX_VALUE) * VDD_LDO_VOLTAGE * VOLTAGE_DIVIDER_RATIO;

#ifndef MY_DISABLED_SERIAL
	Serial.print("ADC raw: ");
	Serial.print(raw);
	Serial.print(" -> ");
	Serial.print(voltage, 3);
	Serial.println("V");
#endif

	return voltage;
}

// Battery cutoff voltage - stop transmitting below this
// With 2.8V LDO, must be above LDO dropout (~200-300mV) = min ~3.0-3.1V battery
#define BATTERY_CUTOFF_V 3.1

uint8_t voltageToBatteryPercent(float voltage)
{
	// Lithium Thionyl Chloride (Li-SOCl2): 3.6V nominal, cutoff at LDO dropout limit
	const float minVoltage = BATTERY_CUTOFF_V;
	const float maxVoltage = 3.6;

	if (voltage <= minVoltage) return 0;
	if (voltage >= maxVoltage) return 100;

	return (uint8_t)(((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100);
}

void loop()
{
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

	// Battery empty - send 0% and stop transmitting
	if (batteryVoltage <= BATTERY_CUTOFF_V) {
		Serial.println("Battery empty! Sending 0% and shutting down.");
		send(msgBatt.set(batteryVoltage, 2));
		sendBatteryLevel(0);
		Serial.flush();
		// Sleep indefinitely to preserve remaining capacity
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

	// Enter low power sleep (MCU STOP mode + radio sleep)
	Serial.println("Sleeping...");
	Serial.flush();

	sleep(SLEEP_TIME);

	Serial.println("Woke up");
}
