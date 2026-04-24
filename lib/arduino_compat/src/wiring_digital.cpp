#include "Arduino.h"
#include "wiring_private.h"

static GPIO_TypeDef* const port_map[] = {
    GPIOA, GPIOB, GPIOC, GPIOD, nullptr, GPIOF,
};

extern "C" GPIO_TypeDef* pin_port(pin_size_t pin)
{
    uint8_t p = (pin >> 4) & 0x0F;
    return (p < sizeof(port_map)/sizeof(port_map[0])) ? port_map[p] : nullptr;
}

extern "C" uint16_t pin_mask(pin_size_t pin)
{
    return (uint16_t)(1u << (pin & 0x0F));
}

extern "C" void pin_enable_port_clock(pin_size_t pin)
{
    switch ((pin >> 4) & 0x0F) {
        case PORT_A: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
        case PORT_B: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
        case PORT_C: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
        case PORT_D: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
        case PORT_F: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
        default: break;
    }
}

extern "C" void pinMode(pin_size_t pin, uint8_t mode)
{
    GPIO_TypeDef* port = pin_port(pin);
    if (!port) return;
    pin_enable_port_clock(pin);

    GPIO_InitTypeDef g = {};
    g.Pin   = pin_mask(pin);
    g.Speed = GPIO_SPEED_FREQ_LOW;
    switch (mode) {
        case INPUT:             g.Mode = GPIO_MODE_INPUT;        g.Pull = GPIO_NOPULL;   break;
        case INPUT_PULLUP:      g.Mode = GPIO_MODE_INPUT;        g.Pull = GPIO_PULLUP;   break;
        case INPUT_PULLDOWN:    g.Mode = GPIO_MODE_INPUT;        g.Pull = GPIO_PULLDOWN; break;
        case OUTPUT:            g.Mode = GPIO_MODE_OUTPUT_PP;    g.Pull = GPIO_NOPULL;   break;
        case OUTPUT_OPEN_DRAIN: g.Mode = GPIO_MODE_OUTPUT_OD;    g.Pull = GPIO_NOPULL;   break;
        case INPUT_ANALOG:      g.Mode = GPIO_MODE_ANALOG;       g.Pull = GPIO_NOPULL;   break;
        default: return;
    }
    HAL_GPIO_Init(port, &g);
}

extern "C" void digitalWrite(pin_size_t pin, uint8_t value)
{
    GPIO_TypeDef* port = pin_port(pin);
    if (!port) return;
    HAL_GPIO_WritePin(port, pin_mask(pin),
                      value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

extern "C" int digitalRead(pin_size_t pin)
{
    GPIO_TypeDef* port = pin_port(pin);
    if (!port) return LOW;
    return (HAL_GPIO_ReadPin(port, pin_mask(pin)) == GPIO_PIN_SET) ? HIGH : LOW;
}
