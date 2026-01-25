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
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 *
 * DESCRIPTION
 * Motion Sensor example using HC-SR501
 * http://www.mysensors.org/build/motion
 *
 */

// Enable debug prints
#define MY_DEBUG
#define MY_SPLASH_SCREEN_DISABLED
#define MY_SMART_SLEEP_WAIT_DURATION_MS 0  // Disable smart sleep wait
#define MY_SLEEP_TRANSPORT_RECONNECT_TIMEOUT_MS 0  // Don't sleep during reconnect

// Enable and select radio type attached
#define MY_RADIO_RFM69          // Radio-Typ
#define MY_RFM69_NEW_DRIVER     // Neuen Driver verwenden (empfohlen)
#define MY_IS_RFM69HW           // Für die HW-Version (High Power)
#define MY_RFM69_FREQUENCY RFM69_868MHZ  // Oder 433MHZ, je nach Modul
#define MY_RFM69_TX_POWER_DBM (6)
#define MY_RFM69_CS_PIN PB6    // Falls nicht Standard-SS
#define MY_RFM69_IRQ_PIN PA10    // Interrupt-Pin
#define MY_RFM69_IRQ_NUM PA10
#define MY_NODE_ID 11          // Feste Node-ID (optional)
#define MY_RFM69_SPI_SPEED (1*1000000ul)	// datasheet says 10Mhz max.

#include <MySensors.h>

// For STOP mode low-power sleep
extern "C" void SystemClock_Config(void);

uint32_t SLEEP_TIME = 120000; // Sleep time between reports (in milliseconds)
// Motion sensor pin - Use PB3 (D19) instead of PA2 (conflicts with ST-Link VCP)
#define DIGITAL_INPUT_SENSOR PB3
#define CHILD_ID 1   // Id of the sensor child

// Initialize motion message
MyMessage msg(CHILD_ID, V_TRIPPED);

volatile bool motionDetected = false;

void motionISR() {
	motionDetected = true;
}

void setup()
{
	// Empty - attach interrupt in loop instead
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("Motion Sensor", "1.0");

	// Register all sensors to gw (they will be created as child devices)
	present(CHILD_ID, S_MOTION);
}

// Forward declaration of MySensors transport functions
extern void transportSleep(void);
extern void transportStandBy(void);

// Enter low-power sleep (SLEEP mode with SysTick disabled + radio sleep)
// Power consumption: ~100-200µA (MCU sleep) + ~0.1µA (RFM69 sleep)
void enterLowPowerSleep()
{
	// Put RFM69 in sleep mode (~0.1µA vs ~1.25mA idle)
	transportSleep();

	// Disable SysTick to prevent 1ms wake-ups
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

	// Clear pending EXTI flags
	EXTI->RPR1 = 0xFFFFFFFF;
	EXTI->FPR1 = 0xFFFFFFFF;

	// Wait for interrupt (CPU sleeps, peripherals running)
	while (!motionDetected) {
		__WFI();
	}

	// Re-enable SysTick
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

	// Radio will wake automatically on next send()
}

void loop()
{
	// Setup interrupt on first run
	static bool interruptAttached = false;
	if (!interruptAttached) {
		interruptAttached = true;
		pinMode(DIGITAL_INPUT_SENSOR, INPUT);
		attachInterrupt(digitalPinToInterrupt(DIGITAL_INPUT_SENSOR), motionISR, CHANGE);
		Serial.println("Interrupt ready on PB3");
	}

	// Read and send motion state
	bool tripped = digitalRead(DIGITAL_INPUT_SENSOR) == HIGH;
	Serial.print("Motion: ");
	Serial.println(tripped);
	send(msg.set(tripped ? "1" : "0"));

	// Clear flag before sleep
	motionDetected = false;
	Serial.println("Sleeping (low power)...");
	Serial.flush();
	delay(10);  // Ensure serial is flushed

	// Enter low-power sleep - waits for GPIO interrupt
	enterLowPowerSleep();

	Serial.println("Woke up!");
}
