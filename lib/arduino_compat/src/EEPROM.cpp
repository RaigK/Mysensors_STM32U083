#include "EEPROM.h"
#include "stm32u0xx_hal.h"
#include <string.h>

EEPROMClass EEPROM;

/* Last 2 KB page of the 256 KB flash (page 127). The address and the page
 * number must match — earlier this used 0x0801F800 (page 63) with EE_PAGE_NUM
 * 127, so HAL_FLASHEx_Erase wiped page 127 while HAL_FLASH_Program targeted
 * page 63. The first save into virgin flash succeeded, every subsequent save
 * silently failed on cells that needed to flip 0→1. */
static constexpr uint32_t EE_FLASH_ADDR = 0x0803F800u;    /* start of page 127 */
static constexpr uint32_t EE_PAGE_NUM   = 127u;           /* last page of bank 1 */

void EEPROMClass::load_()
{
    if (_loaded) return;
    memcpy(_shadow, (const void*)EE_FLASH_ADDR, SIZE);
    _loaded = true;
    _dirty  = false;
}

void EEPROMClass::flush_()
{
    if (!_dirty) return;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = EE_PAGE_NUM;
    erase.NbPages   = 1;
    uint32_t page_err = 0;
    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    /* STM32U0 flash programs in 64-bit (doubleword) quanta. */
    for (uint32_t i = 0; i < SIZE; i += 8) {
        uint64_t dw = 0;
        memcpy(&dw, &_shadow[i], 8);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              EE_FLASH_ADDR + i, dw) != HAL_OK) {
            break;
        }
    }

    HAL_FLASH_Lock();
    _dirty = false;
}

uint8_t EEPROMClass::read(int addr)
{
    if (addr < 0 || addr >= SIZE) return 0xFF;
    load_();
    return _shadow[addr];
}

void EEPROMClass::write(int addr, uint8_t v)
{
    if (addr < 0 || addr >= SIZE) return;
    load_();
    if (_shadow[addr] != v) {
        _shadow[addr] = v;
        _dirty = true;
        flush_();
    }
}

void EEPROMClass::update(int addr, uint8_t v)
{
    write(addr, v);   /* write already short-circuits when value matches */
}
