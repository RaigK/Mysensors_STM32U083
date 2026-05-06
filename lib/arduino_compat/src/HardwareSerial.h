#pragma once
#include "Stream.h"
#include "stm32u0xx_hal.h"
#include "pins_arduino.h"

/*
 * Minimal blocking serial over a single UART peripheral.
 * RX buffering is interrupt-driven via a small ring buffer.
 */
class HardwareSerial : public Stream {
public:
    HardwareSerial(USART_TypeDef* inst) : _inst(inst) {}

    void begin(unsigned long baud);
    void begin(unsigned long baud, uint8_t config);  /* config ignored — always 8N1 */
    void end();

    /* STM32duino-compat no-ops. Our USART2 is hardwired to PA2/PA3 AF7. */
    void setTx(pin_size_t) {}
    void setRx(pin_size_t) {}

    int  available() override;
    int  read() override;
    int  peek() override;
    size_t write(uint8_t c) override;
    using Print::write;
    void flush() override;

    /* Called by the IRQ handler. Public so the extern "C" handler can reach it. */
    void _handleIRQ();

    operator bool() const { return _ready; }

private:
    USART_TypeDef*   _inst;
    UART_HandleTypeDef _huart = {};
    bool _ready = false;

    static constexpr size_t RX_BUF = 64;
    volatile uint8_t  _rxbuf[RX_BUF];
    volatile uint16_t _rx_head = 0;
    volatile uint16_t _rx_tail = 0;
};

/* USART2 on PA2/PA3 AF7 — what the existing firmware uses for debug. */
extern HardwareSerial Serial;
