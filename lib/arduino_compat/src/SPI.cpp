#include "SPI.h"
#include "Arduino.h"

SPIClass SPI;

/* APB1 prescaler → pick BAUDRATEPRESCALER closest to requested clock. */
static uint32_t clock_to_prescaler(uint32_t requested_hz)
{
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    static const uint32_t presc[] = {
        SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
        SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
        SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
    };
    for (uint8_t i = 0; i < 8; ++i) {
        if ((pclk >> (i + 1)) <= requested_hz) return presc[i];
    }
    return SPI_BAUDRATEPRESCALER_256;
}

void SPIClass::applySettings_(uint32_t clock, uint8_t mode, uint8_t order)
{
    if (_ready &&
        clock == _current_clock &&
        mode  == _current_mode  &&
        order == _current_order) return;

    if (_ready) HAL_SPI_DeInit(&_hspi);

    _hspi.Instance               = SPI1;
    _hspi.Init.Mode              = SPI_MODE_MASTER;
    _hspi.Init.Direction         = SPI_DIRECTION_2LINES;
    _hspi.Init.DataSize          = SPI_DATASIZE_8BIT;
    _hspi.Init.CLKPolarity       = (mode & 0x02) ? SPI_POLARITY_HIGH    : SPI_POLARITY_LOW;
    _hspi.Init.CLKPhase          = (mode & 0x01) ? SPI_PHASE_2EDGE      : SPI_PHASE_1EDGE;
    _hspi.Init.NSS               = SPI_NSS_SOFT;
    _hspi.Init.BaudRatePrescaler = clock_to_prescaler(clock);
    _hspi.Init.FirstBit          = (order == LSBFIRST) ? SPI_FIRSTBIT_LSB : SPI_FIRSTBIT_MSB;
    _hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
    _hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    _hspi.Init.CRCPolynomial     = 7;
    _hspi.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&_hspi) != HAL_OK) { _ready = false; return; }

    _ready         = true;
    _current_clock = clock;
    _current_mode  = mode;
    _current_order = order;
}

void SPIClass::begin()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    GPIO_InitTypeDef g = {};
    g.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;   /* SCK, MISO, MOSI */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &g);

    applySettings_(1000000, SPI_MODE0, MSBFIRST);
}

void SPIClass::end()
{
    if (_ready) { HAL_SPI_DeInit(&_hspi); _ready = false; }
    __HAL_RCC_SPI1_CLK_DISABLE();
}

void SPIClass::beginTransaction(const SPISettings& s)
{
    applySettings_(s.clock, s.dataMode, s.bitOrder);
}

uint8_t SPIClass::transfer(uint8_t b)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&_hspi, &b, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

uint16_t SPIClass::transfer16(uint16_t v)
{
    uint8_t tx[2] = { (uint8_t)(v >> 8), (uint8_t)(v & 0xFF) };
    uint8_t rx[2] = {};
    HAL_SPI_TransmitReceive(&_hspi, tx, rx, 2, HAL_MAX_DELAY);
    return ((uint16_t)rx[0] << 8) | rx[1];
}

void SPIClass::transfer(void* buf, size_t count)
{
    uint8_t* p = (uint8_t*)buf;
    HAL_SPI_TransmitReceive(&_hspi, p, p, (uint16_t)count, HAL_MAX_DELAY);
}

void SPIClass::setClockDivider(uint8_t div)
{
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    uint32_t clock = pclk / (div ? div : 2);
    applySettings_(clock, _current_mode, _current_order);
}

void SPIClass::setBitOrder(uint8_t order)
{
    applySettings_(_current_clock, _current_mode, order);
}

void SPIClass::setDataMode(uint8_t mode)
{
    applySettings_(_current_clock, mode, _current_order);
}
