#pragma once
/*
 * AVR PROGMEM is a no-op on Cortex-M (unified address space).
 * Upstream MySensors and its AES driver unconditionally include this header.
 */
#include "../Arduino.h"
