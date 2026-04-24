#pragma once
#include <stdint.h>
#include "stm32u0xx_hal.h"

#define SPI_HAS_TRANSACTION 1

#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

#define LSBFIRST 0
#define MSBFIRST 1

class SPISettings {
public:
    SPISettings() : clock(1000000), bitOrder(MSBFIRST), dataMode(SPI_MODE0) {}
    SPISettings(uint32_t c, uint8_t bo, uint8_t dm) : clock(c), bitOrder(bo), dataMode(dm) {}
    uint32_t clock;
    uint8_t  bitOrder;
    uint8_t  dataMode;
};

/*
 * SPI1 on PA5 (SCK), PA6 (MISO), PA7 (MOSI), AF5. CS is caller-managed.
 * Blocking, polled transfers. No DMA.
 */
class SPIClass {
public:
    void begin();
    void end();

    void beginTransaction(const SPISettings& s);
    void endTransaction() { /* no-op */ }

    uint8_t  transfer(uint8_t b);
    uint16_t transfer16(uint16_t v);
    void     transfer(void* buf, size_t count);

    /* Legacy setters. Implemented via a fresh Init. */
    void setClockDivider(uint8_t div);
    void setBitOrder(uint8_t order);
    void setDataMode(uint8_t mode);

    SPI_HandleTypeDef* handle() { return &_hspi; }

private:
    SPI_HandleTypeDef _hspi = {};
    bool     _ready = false;
    uint32_t _current_clock = 0;
    uint8_t  _current_mode  = SPI_MODE0;
    uint8_t  _current_order = MSBFIRST;

    void applySettings_(uint32_t clock, uint8_t mode, uint8_t order);
};

extern SPIClass SPI;
