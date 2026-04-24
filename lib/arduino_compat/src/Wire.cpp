#include "Wire.h"

TwoWire Wire;

/*
 * I2C TIMINGR value. On STM32U0 this is computed from the peripheral input
 * clock plus the desired SCL. CubeMX provides a calculator, but the mapping
 * at PCLK = 16 MHz is well-known and covers our two operating points.
 *   - 100 kHz standard
 *   - 400 kHz fast
 *   - 10  kHz  — value used by existing firmware for HDC1080 reliability
 */
static uint32_t i2c_timing_for(uint32_t clock_hz)
{
    /* Values taken from STM32CubeMX at PCLK1 = 16 MHz, AnalogFilter on. */
    if (clock_hz >= 400000) return 0x00300619;  /* 400 kHz */
    if (clock_hz >=  80000) return 0x00503D5B;  /* 100 kHz */
    return 0x00000E14;                           /* ~10 kHz (fallback slow) */
}

void TwoWire::applyClock_()
{
    _hi2c.Instance              = I2C2;
    _hi2c.Init.Timing           = i2c_timing_for(_clock_hz);
    _hi2c.Init.OwnAddress1      = 0;
    _hi2c.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    _hi2c.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    _hi2c.Init.OwnAddress2      = 0;
    _hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    _hi2c.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    _hi2c.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&_hi2c) != HAL_OK) { _ready = false; return; }
    HAL_I2CEx_ConfigAnalogFilter(&_hi2c, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&_hi2c, 0);
    _ready = true;
}

void TwoWire::begin()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    GPIO_InitTypeDef g = {};
    g.Pin       = GPIO_PIN_10 | GPIO_PIN_11;   /* SCL, SDA */
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &g);

    applyClock_();
}

void TwoWire::end()
{
    if (_ready) { HAL_I2C_DeInit(&_hi2c); _ready = false; }
    __HAL_RCC_I2C2_CLK_DISABLE();
}

void TwoWire::setClock(uint32_t hz)
{
    _clock_hz = hz;
    if (_ready) applyClock_();
}

void TwoWire::beginTransmission(uint8_t address)
{
    _tx_addr = address;
    _tx_len  = 0;
}

size_t TwoWire::write(uint8_t b)
{
    if (_tx_len >= BUF) return 0;
    _tx[_tx_len++] = b;
    return 1;
}

size_t TwoWire::write(const uint8_t* buf, size_t len)
{
    size_t n = 0;
    while (len--) { if (!write(*buf++)) break; ++n; }
    return n;
}

uint8_t TwoWire::endTransmission(bool sendStop)
{
    (void)sendStop;  /* HAL_I2C_Master_Transmit always issues stop */
    if (!_ready) return 4;

    HAL_StatusTypeDef s = HAL_I2C_Master_Transmit(
        &_hi2c,
        (uint16_t)(_tx_addr << 1),
        _tx, (uint16_t)_tx_len,
        HAL_MAX_DELAY);

    _tx_len = 0;
    /* Arduino codes: 0 = ok, 2 = addr NACK, 3 = data NACK, 4 = other. */
    switch (s) {
        case HAL_OK:      return 0;
        case HAL_TIMEOUT: return 5;
        default:          return 4;
    }
}

size_t TwoWire::requestFrom(uint8_t address, size_t count, bool sendStop)
{
    (void)sendStop;
    if (!_ready || count > BUF) return 0;

    HAL_StatusTypeDef s = HAL_I2C_Master_Receive(
        &_hi2c,
        (uint16_t)(address << 1),
        _rx, (uint16_t)count,
        HAL_MAX_DELAY);

    if (s != HAL_OK) { _rx_len = 0; _rx_pos = 0; return 0; }
    _rx_len = count;
    _rx_pos = 0;
    return count;
}

int TwoWire::available()
{
    return (int)(_rx_len - _rx_pos);
}

int TwoWire::read()
{
    if (_rx_pos >= _rx_len) return -1;
    return _rx[_rx_pos++];
}

int TwoWire::peek()
{
    if (_rx_pos >= _rx_len) return -1;
    return _rx[_rx_pos];
}

size_t TwoWire::readBytes(uint8_t* buf, size_t len)
{
    size_t n = 0;
    while (n < len && _rx_pos < _rx_len) { buf[n++] = _rx[_rx_pos++]; }
    return n;
}
