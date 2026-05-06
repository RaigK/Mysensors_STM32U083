#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

#include "stm32u0xx_hal.h"

#include "wiring_constants.h"
#include "pins_arduino.h"

/* STM32duino-compat: alternate pin-function aliases used by existing sketch.
 * We hardcode USART2 to PA2/PA3 AF7 and I2C2 to PB10/PB11 AF4, so the ALT1
 * variants resolve to the same physical pins. */
#define PA_2_ALT1 PA2
#define PA_3_ALT1 PA3

/*
 * PROGMEM compatibility for Cortex-M. Flash and RAM share the address space,
 * so these are all direct pointer reads / trivial passthroughs.
 */
#define PROGMEM
#define PGM_P       const char*
#define PGM_VOID_P  const void*
#ifndef PSTR
#define PSTR(x) x
#endif

#ifdef __cplusplus
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))
#else
#define F(s) (s)
#endif

#define pgm_read_byte(addr)   (*(const uint8_t*)(addr))
#define pgm_read_word(addr)   (*(const uint16_t*)(addr))
#define pgm_read_dword(addr)  (*(const uint32_t*)(addr))
#define pgm_read_float(addr)  (*(const float*)(addr))
#define pgm_read_ptr(addr)    (*(const void* const*)(addr))

#define pgm_read_byte_near(addr)  pgm_read_byte(addr)
#define pgm_read_word_near(addr)  pgm_read_word(addr)
#define pgm_read_dword_near(addr) pgm_read_dword(addr)

#define memcpy_P    memcpy
#define memcmp_P    memcmp
#define strcpy_P    strcpy
#define strncpy_P   strncpy
#define strcmp_P    strcmp
#define strncmp_P   strncmp
#define strcasecmp_P strcasecmp
#define strlen_P    strlen
#define strstr_P    strstr
#define sprintf_P   sprintf
#define snprintf_P  snprintf
#define vsnprintf_P vsnprintf
#define printf_P    printf
#define fprintf_P   fprintf
#define vprintf_P   vprintf

#ifdef __cplusplus
extern "C" {
#endif
void Error_Handler(void);
#ifdef __cplusplus
}
#endif

/* Arduino stdlib extensions MySensors reaches for. */
#ifdef __cplusplus
extern "C" {
#endif
char* ultoa(unsigned long v, char* s, int base);
char* ltoa(long v, char* s, int base);
char* itoa(int v, char* s, int base);
char* utoa(unsigned int v, char* s, int base);
char* dtostrf(double v, signed char width, unsigned char prec, char* s);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* wiring_digital */
void pinMode(pin_size_t pin, uint8_t mode);
void digitalWrite(pin_size_t pin, uint8_t value);
int  digitalRead(pin_size_t pin);

/* wiring_time */
uint32_t millis(void);
uint32_t micros(void);
void     delay(uint32_t ms);
void     delayMicroseconds(uint32_t us);
void     yield(void);

/* Interrupts */
#define interrupts()    __enable_irq()
#define noInterrupts()  __disable_irq()

typedef void (*voidFuncPtr)(void);
void attachInterrupt(pin_size_t pin, voidFuncPtr cb, int mode);
void detachInterrupt(pin_size_t pin);

/* Arduino typedefs */
typedef uint8_t  byte;
typedef bool     boolean;
typedef uint16_t word;

/* User application entry points (Arduino convention). */
void setup(void);
void loop(void);

/* Framework hooks used by MySensors' MyMainSTM32.cpp. */
void init(void);
void initVariant(void);
void serialEventRun(void);
void SystemClock_Config(void);

/* Analog helpers — project code uses direct HAL, but upstream MyHwSTM32.cpp
 * calls analogRead() for the internal temperature sensor. Stubs keep the
 * code compiling; hwCPUTemperature() will just return 0. */
int  analogRead(pin_size_t pin);
void analogReadResolution(int bits);
void analogWrite(pin_size_t pin, int value);
void analogReference(uint8_t mode);
#define ATEMP 0xFE   /* dummy pin id for the internal temp sensor  */
#define AVREF 0xFD   /* dummy pin id for the internal VREFINT ref   */

/* randomSeed is C-linkage safe. The Arduino random() overloads conflict with
 * libc's zero-arg random() in C mode, so they live outside this extern "C"
 * block further down. */
void randomSeed(unsigned long seed);

#ifdef __cplusplus
}  /* extern "C" */

/* Arduino-style random() overloads. Only valid in C++; the zero-arg libc
 * random() keeps its original signature in C code. */
long random(long max);
long random(long min, long max);

#include "Print.h"
#include "Stream.h"
#include "HardwareSerial.h"
#include "SPI.h"
#include "Wire.h"

#endif

/* Helpers commonly pulled from Arduino.h */
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#ifndef abs
#define abs(x) ((x)>0?(x):-(x))
#endif
#ifndef constrain
#define constrain(x,lo,hi) ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
#endif

#define bitRead(value, bit)     (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)      ((value) |=  (1UL << (bit)))
#define bitClear(value, bit)    ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, v) ((v) ? bitSet(value, bit) : bitClear(value, bit))

#define _BV(b) (1UL << (b))
