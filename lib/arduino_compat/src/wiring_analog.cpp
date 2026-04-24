/*
 * Stubs for Arduino analog I/O. Real ADC work in this project goes through
 * readADC_U0() in src/main.cpp using HAL directly. These stubs exist only
 * to keep MyHwSTM32.cpp (which calls analogRead(ATEMP)) compiling.
 */
#include "Arduino.h"

extern "C" int  analogRead(pin_size_t pin)          { (void)pin; return 0; }
extern "C" void analogReadResolution(int bits)      { (void)bits; }
extern "C" void analogWrite(pin_size_t pin, int v)  { (void)pin; (void)v; }
extern "C" void analogReference(uint8_t mode)       { (void)mode; }

/* random() / randomSeed() — Arduino semantics over libc rand. */
long random(long max)            { return (max > 0) ? (long)(rand() % max) : 0; }
long random(long min, long max)  { return (max > min) ? (min + (long)(rand() % (max - min))) : min; }
extern "C" void randomSeed(unsigned long s) { srand((unsigned)s); }
