#include "STM32RTC.h"

STM32RTC& STM32RTC::getInstance()
{
    static STM32RTC inst;
    return inst;
}

void STM32RTC::setClockSource(Source_Clock src)
{
    _src = src;
}

void STM32RTC::begin(bool /*resetTime*/)
{
    if (_initialized) return;

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    /* LSI on (32 kHz), routed to RTC. */
    RCC_OscInitTypeDef osc = {};
    osc.OscillatorType = RCC_OSCILLATORTYPE_LSI;
    osc.LSIState       = RCC_LSI_ON;
    osc.PLL.PLLState   = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&osc);

    RCC_PeriphCLKInitTypeDef pclk = {};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    pclk.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
    HAL_RCCEx_PeriphCLKConfig(&pclk);

    __HAL_RCC_RTC_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();

    _hrtc.Instance            = RTC;
    _hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    _hrtc.Init.AsynchPrediv   = 127;
    _hrtc.Init.SynchPrediv    = 249;
    _hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    _hrtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
    _hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    _hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    _hrtc.Init.OutPutPullUp   = RTC_OUTPUT_PULLUP_NONE;

    HAL_RTC_Init(&_hrtc);
    _initialized = true;
}

uint8_t STM32RTC::getHours()
{
    RTC_TimeTypeDef t = {};
    RTC_DateTypeDef d = {};
    HAL_RTC_GetTime(&_hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&_hrtc, &d, RTC_FORMAT_BIN);  /* required to unlock time shadow */
    return t.Hours;
}

uint8_t STM32RTC::getMinutes()
{
    RTC_TimeTypeDef t = {};
    RTC_DateTypeDef d = {};
    HAL_RTC_GetTime(&_hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&_hrtc, &d, RTC_FORMAT_BIN);
    return t.Minutes;
}

uint8_t STM32RTC::getSeconds()
{
    RTC_TimeTypeDef t = {};
    RTC_DateTypeDef d = {};
    HAL_RTC_GetTime(&_hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&_hrtc, &d, RTC_FORMAT_BIN);
    return t.Seconds;
}

/*
 * Alarm A support.
 *
 * STM32duino's RTC library owns RTC_TAMP_IRQHandler on STM32U0 and routes
 * Alarm A events to a user callback via HAL_RTC_AlarmAEventCallback. We have
 * no STM32duino RTC in the cube env, so we provide the strong IRQ handler and
 * the Alarm A event callback here. The user callback is stored as file-locals
 * because the IRQ handler is plain C.
 */
static STM32RTC::voidCallbackPtr s_alarmA_cb   = nullptr;
static void*                     s_alarmA_data = nullptr;

void STM32RTC::attachInterrupt(voidCallbackPtr cb, void* data, Alarm_Source src)
{
    if (src != ALARM_A) return;
    s_alarmA_cb   = cb;
    s_alarmA_data = data;
    HAL_NVIC_SetPriority(RTC_TAMP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_TAMP_IRQn);
}

void STM32RTC::disableAlarm(Alarm_Source src)
{
    if (src != ALARM_A) return;
    HAL_RTC_DeactivateAlarm(&_hrtc, RTC_ALARM_A);
}

extern "C" void RTC_TAMP_IRQHandler(void)
{
    HAL_RTC_AlarmIRQHandler(STM32RTC::getInstance().getHandle());
}

extern "C" void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef* /*hrtc*/)
{
    if (s_alarmA_cb) s_alarmA_cb(s_alarmA_data);
}
