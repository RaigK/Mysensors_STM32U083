#pragma once
#include <stdint.h>
#include <stddef.h>

/*
 * Flash-emulated EEPROM for STM32U083.
 *
 * Uses the last 2 KB page of flash (page 127 @ 0x0801F800..0x0801FFFF) as a
 * single shadowed buffer. Reads hit a RAM shadow; writes update the shadow
 * and flush the whole page (erase + reprogram) immediately.
 *
 * Immediate flush is slow (~20 ms erase + ~2 ms program) and will stall ISRs
 * during that window. This matches STM32duino's EEPROM semantics — the
 * application (MyHwSTM32.cpp / main.cpp deferred flushPendingSaves pattern)
 * is responsible for batching writes.
 */
class EEPROMClass {
public:
    static constexpr uint16_t SIZE = 2048;

    uint8_t  read(int addr);
    void     write(int addr, uint8_t v);
    void     update(int addr, uint8_t v);
    uint16_t length() const { return SIZE; }

private:
    void load_();
    void flush_();

    uint8_t  _shadow[SIZE];
    bool     _loaded = false;
    bool     _dirty  = false;
};

extern EEPROMClass EEPROM;
