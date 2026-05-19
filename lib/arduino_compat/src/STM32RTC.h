#pragma once
#include <stdint.h>
#include "stm32u0xx_hal.h"

/*
 * Thin shim for the STM32duino `STM32RTC` singleton used by the patched
 * MyHwSTM32.cpp. Only the subset of the API that MyHwSTM32.cpp calls is
 * implemented. LSI is the only clock source supported (matches existing
 * behaviour).
 */
class STM32RTC {
public:
    enum Source_Clock { LSE_CLOCK, HSE_CLOCK, LSI_CLOCK };
    enum Alarm_Source { ALARM_A, ALARM_B };

    typedef void (*voidCallbackPtr)(void*);

    static STM32RTC& getInstance();

    void setClockSource(Source_Clock src);
    void begin(bool resetTime = false);

    RTC_HandleTypeDef* getHandle() { return &_hrtc; }

    uint8_t getHours();
    uint8_t getMinutes();
    uint8_t getSeconds();

    // Alarm support. The cube env has no STM32duino RTC library, so we
    // own the strong RTC_TAMP_IRQHandler and HAL_RTC_AlarmAEventCallback
    // here. attachInterrupt registers a user callback that fires from the
    // HAL callback whenever Alarm A matches.
    void attachInterrupt(voidCallbackPtr cb, void* data = nullptr,
                         Alarm_Source src = ALARM_A);
    void disableAlarm(Alarm_Source src = ALARM_A);

private:
    STM32RTC() = default;
    RTC_HandleTypeDef _hrtc = {};
    Source_Clock      _src  = LSI_CLOCK;
    bool              _initialized = false;
};
