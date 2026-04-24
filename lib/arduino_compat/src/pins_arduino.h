#pragma once

/*
 * Pin numbering: (port_index << 4) | pin_in_port
 *   port_index: 0=A, 1=B, 2=C, 3=D, 5=F   (U083 has no GPIOE/G/H)
 * So PA0=0x00, PA5=0x05, PB0=0x10, PB6=0x16, PC13=0x2D.
 */
#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t pin_size_t;

#define PORT_A  0
#define PORT_B  1
#define PORT_C  2
#define PORT_D  3
#define PORT_F  5

#define PIN(port, n)  ((pin_size_t)(((port) << 4) | ((n) & 0x0F)))

/* All pins we use on this board, plus a superset covering typical needs. */
#define PA0   PIN(PORT_A, 0)
#define PA1   PIN(PORT_A, 1)
#define PA2   PIN(PORT_A, 2)
#define PA3   PIN(PORT_A, 3)
#define PA4   PIN(PORT_A, 4)
#define PA5   PIN(PORT_A, 5)
#define PA6   PIN(PORT_A, 6)
#define PA7   PIN(PORT_A, 7)
#define PA8   PIN(PORT_A, 8)
#define PA9   PIN(PORT_A, 9)
#define PA10  PIN(PORT_A, 10)
#define PA11  PIN(PORT_A, 11)
#define PA12  PIN(PORT_A, 12)
#define PA13  PIN(PORT_A, 13)
#define PA14  PIN(PORT_A, 14)
#define PA15  PIN(PORT_A, 15)

#define PB0   PIN(PORT_B, 0)
#define PB1   PIN(PORT_B, 1)
#define PB2   PIN(PORT_B, 2)
#define PB3   PIN(PORT_B, 3)
#define PB4   PIN(PORT_B, 4)
#define PB5   PIN(PORT_B, 5)
#define PB6   PIN(PORT_B, 6)
#define PB7   PIN(PORT_B, 7)
#define PB8   PIN(PORT_B, 8)
#define PB9   PIN(PORT_B, 9)
#define PB10  PIN(PORT_B, 10)
#define PB11  PIN(PORT_B, 11)
#define PB12  PIN(PORT_B, 12)
#define PB13  PIN(PORT_B, 13)
#define PB14  PIN(PORT_B, 14)
#define PB15  PIN(PORT_B, 15)

#define PC0   PIN(PORT_C, 0)
#define PC13  PIN(PORT_C, 13)
#define PC14  PIN(PORT_C, 14)
#define PC15  PIN(PORT_C, 15)

#define NUM_DIGITAL_PINS 96

/* EXTI line == pin_in_port on STM32. digitalPinToInterrupt is identity. */
#define digitalPinToInterrupt(p) ((p) & 0x0F)

#define NOT_A_PIN 0xFF

#ifdef __cplusplus
}
#endif
