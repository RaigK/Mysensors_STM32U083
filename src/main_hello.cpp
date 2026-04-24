/*
 * Phase 2 checkpoint: Arduino shim over CubeU0 HAL.
 * Blinks PB0 at 1 Hz and prints "tick N" over USART2 (COM9, 115200 8N1).
 */
#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== cube shim hello ===");
    Serial.print("SystemCoreClock = ");
    Serial.print((unsigned long)SystemCoreClock);
    Serial.println(" Hz");

    pinMode(PB0, OUTPUT);
    digitalWrite(PB0, LOW);
}

void loop()
{
    static uint32_t n = 0;
    digitalWrite(PB0, HIGH);
    delay(500);
    digitalWrite(PB0, LOW);
    delay(500);
    Serial.print("tick ");
    Serial.println(n++);
}
