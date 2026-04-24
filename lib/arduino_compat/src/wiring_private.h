#pragma once
#include <stdint.h>
#include "stm32u0xx_hal.h"
#include "pins_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Translate packed pin id → (port, mask). Enables the port clock on demand. */
GPIO_TypeDef* pin_port(pin_size_t pin);
uint16_t      pin_mask(pin_size_t pin);
void          pin_enable_port_clock(pin_size_t pin);

#ifdef __cplusplus
}
#endif
