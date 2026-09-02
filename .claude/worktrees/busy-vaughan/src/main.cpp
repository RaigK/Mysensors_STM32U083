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
#define MY_STM32_RUN_MODE    POWER_RUN_LOW_POWER   // Low Power Run 2 MHz
#define MY_STM32_SLEEP_MODE  POWER_SLEEP_STOP2     // Stop 2 mode (~1-3 µA)
#include "stm32_power_config.h"

// Enable debug prints (comment out for lowest power)
// #define MY_DEBUG
#define MY_SPLASH_SCREEN_DISABLED
#define MY_DISABLED_SERIAL  // Disable serial for lowest power consumption
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
#define BATTERY_ADC_PIN PA0
// Voltage divider ratio (if using one) - adjust based on your circuit
// For direct connection to battery (max 3.3V): set to 1.0
// For voltage divider (e.g., 100k/100k for max 6.6V): set to 2.0
#define VOLTAGE_DIVIDER_RATIO 2.0
// ADC resolution (12-bit)
#define ADC_MAX_VALUE 4095

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

	// Voltage scaling range 2 for lower power (valid up to 16 MHz)
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

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
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
#endif
}

void setup()
{
	// Early debug - before MySensors init
	Serial.begin(115200);
	delay(100);
	Serial.println("\n\n=== BOOT ===");
	Serial.flush();

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

	// Configure battery ADC pin (12-bit to match ADC_MAX_VALUE 4095)
	analogReadResolution(12);
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

float readVDDA()
{
	// Measure actual VDDA using factory-calibrated internal voltage reference
	uint32_t vrefSum = 0;
	const int numReadings = 10;

	for (int i = 0; i < numReadings; i++) {
		vrefSum += analogRead(AVREF);
		delay(1);
	}

	float vrefintRaw = (float)vrefSum / numReadings;

	// VDDA = calibration_voltage_mV * calibration_value / current_reading / 1000
	return ((float)VREFINT_CAL_VREF * (float)(*VREFINT_CAL_ADDR) / vrefintRaw) / 1000.0f;
}

float readBatteryVoltage()
{
	// Read ADC value (average of multiple readings for stability)
	uint32_t adcSum = 0;
	const int numReadings = 10;

	for (int i = 0; i < numReadings; i++) {
		adcSum += analogRead(BATTERY_ADC_PIN);
		delay(1);
	}

	float adcValue = (float)adcSum / numReadings;

	// Convert to voltage using measured VDDA instead of hardcoded 3.3V
	float vdda = readVDDA();
	float voltage = (adcValue / ADC_MAX_VALUE) * vdda * VOLTAGE_DIVIDER_RATIO;

	return voltage;
}

// Battery cutoff voltage - stop transmitting below this
#define BATTERY_CUTOFF_V 2.85

uint8_t voltageToBatteryPercent(float voltage)
{
	// Lithium Thionyl Chloride (Li-SOCl2): 3.6V nominal, 2.85V cutoff
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
