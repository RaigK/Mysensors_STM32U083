/*
 * attachInterrupt / detachInterrupt over STM32U0 EXTI lines 0..15.
 *
 * Pin 0x0A (PA10) → EXTI line 10, SYSCFG EXTICR2 field 2 selected to port A.
 * STM32U0 combines EXTI vectors: EXTI0_1 / EXTI2_3 / EXTI4_15.
 * STM32U0 uses separate Rising/Falling pending registers (RPR1 / FPR1).
 */
#include "Arduino.h"
#include "wiring_private.h"

typedef void (*voidFuncPtr)(void);

static voidFuncPtr s_handlers[16] = {};

static IRQn_Type irq_for_line(uint8_t line)
{
    if (line <= 1) return EXTI0_1_IRQn;
    if (line <= 3) return EXTI2_3_IRQn;
    return EXTI4_15_IRQn;
}

/*
 * SYSCFG_EXTICR maps which port (A/B/C/D/F) drives a given EXTI line.
 * On U0 this lives in EXTI->EXTICR[0..3] (not SYSCFG as on older families).
 */
static void route_line_to_port(uint8_t line, uint8_t port_index)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    uint32_t reg = line >> 2;       /* 0..3 */
    uint32_t shift = (line & 3) * 8;
    EXTI->EXTICR[reg] = (EXTI->EXTICR[reg] & ~(0xFFu << shift)) |
                        ((uint32_t)port_index << shift);
}

extern "C" void attachInterrupt(pin_size_t pin, voidFuncPtr callback, int mode)
{
    uint8_t port_idx = (pin >> 4) & 0x0F;
    uint8_t line     =  pin & 0x0F;

    s_handlers[line] = callback;
    pin_enable_port_clock(pin);

    /* Pin as input (EXTI needs the input buffer enabled). */
    GPIO_TypeDef* port = pin_port(pin);
    if (port) {
        GPIO_InitTypeDef g = {};
        g.Pin   = pin_mask(pin);
        g.Speed = GPIO_SPEED_FREQ_LOW;
        g.Pull  = GPIO_NOPULL;
        switch (mode) {
            case RISING:  g.Mode = GPIO_MODE_IT_RISING;         break;
            case FALLING: g.Mode = GPIO_MODE_IT_FALLING;        break;
            case CHANGE:  g.Mode = GPIO_MODE_IT_RISING_FALLING; break;
            default:      g.Mode = GPIO_MODE_IT_RISING;         break;
        }
        HAL_GPIO_Init(port, &g);
    }

    route_line_to_port(line, port_idx);

    HAL_NVIC_SetPriority(irq_for_line(line), 2, 0);
    HAL_NVIC_EnableIRQ(irq_for_line(line));
}

extern "C" void detachInterrupt(pin_size_t pin)
{
    uint8_t line = pin & 0x0F;
    s_handlers[line] = nullptr;
    EXTI->IMR1  &= ~(1u << line);
    EXTI->RTSR1 &= ~(1u << line);
    EXTI->FTSR1 &= ~(1u << line);
}

/* Dispatcher for one line. Clears pending and invokes the handler. */
static inline void dispatch_line(uint8_t line)
{
    uint32_t bit = 1u << line;
    bool fired = false;
    if (EXTI->RPR1 & bit) { EXTI->RPR1 = bit; fired = true; }
    if (EXTI->FPR1 & bit) { EXTI->FPR1 = bit; fired = true; }
    if (fired && s_handlers[line]) s_handlers[line]();
}

extern "C" void EXTI0_1_IRQHandler(void)
{
    dispatch_line(0);
    dispatch_line(1);
}

extern "C" void EXTI2_3_IRQHandler(void)
{
    dispatch_line(2);
    dispatch_line(3);
}

extern "C" void EXTI4_15_IRQHandler(void)
{
    for (uint8_t i = 4; i < 16; ++i) dispatch_line(i);
}
