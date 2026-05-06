#pragma once
#include <stdint.h>
#include <stddef.h>
#include "stm32u0xx_hal.h"

/*
 * I2C2 on PB10 (SCL), PB11 (SDA), AF4. Master only. Blocking HAL transfers.
 * Buffers sized for typical MySensors / HDC1080 use (small registers, ≤ 32 B).
 */
class TwoWire {
public:
    void begin();
    void end();
    void setClock(uint32_t hz);

    /* STM32duino-compat no-ops. I2C2 is hardwired to PB10/PB11 AF4. */
    void setSCL(uint8_t) {}
    void setSDA(uint8_t) {}

    void beginTransmission(uint8_t address);
    size_t write(uint8_t b);
    size_t write(const uint8_t* buf, size_t len);
    uint8_t endTransmission(bool sendStop = true);

    size_t requestFrom(uint8_t address, size_t count, bool sendStop = true);
    int available();
    int read();
    int peek();

    /* Stream::readBytes compatibility — HDC1080 library expects it. */
    size_t readBytes(uint8_t* buf, size_t len);
    size_t readBytes(char*    buf, size_t len) { return readBytes((uint8_t*)buf, len); }

private:
    I2C_HandleTypeDef _hi2c = {};
    bool     _ready    = false;
    uint32_t _clock_hz = 100000;

    static constexpr size_t BUF = 32;
    uint8_t  _tx[BUF];
    size_t   _tx_len = 0;
    uint8_t  _tx_addr = 0;

    uint8_t  _rx[BUF];
    size_t   _rx_len = 0;
    size_t   _rx_pos = 0;

    void applyClock_();
};

extern TwoWire Wire;
