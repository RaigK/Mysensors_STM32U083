#include "HardwareSerial.h"

/*
 * Only USART2 is wired for now — extend later if we need more.
 * PA2 = TX (AF7), PA3 = RX (AF7).
 */
HardwareSerial Serial(USART2);
static HardwareSerial* s_usart2_instance = nullptr;

void HardwareSerial::begin(unsigned long baud)
{
    begin(baud, 0);
}

void HardwareSerial::begin(unsigned long baud, uint8_t /*config*/)
{
    if (_ready) end();

    /* Route clocks + pins for USART2. */
    if (_inst == USART2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();

        GPIO_InitTypeDef g = {};
        g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_PULLUP;
        g.Speed     = GPIO_SPEED_FREQ_LOW;
        g.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &g);

        s_usart2_instance = this;
    } else {
        return;
    }

    _huart.Instance                    = _inst;
    _huart.Init.BaudRate               = baud;
    _huart.Init.WordLength             = UART_WORDLENGTH_8B;
    _huart.Init.StopBits               = UART_STOPBITS_1;
    _huart.Init.Parity                 = UART_PARITY_NONE;
    _huart.Init.Mode                   = UART_MODE_TX_RX;
    _huart.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    _huart.Init.OverSampling           = UART_OVERSAMPLING_16;
    _huart.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    _huart.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    _huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&_huart) != HAL_OK) return;

    /* Enable RXNE interrupt for receive-into-ring-buffer. */
    if (_inst == USART2) {
        HAL_NVIC_SetPriority(USART2_LPUART2_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART2_LPUART2_IRQn);
    }
    __HAL_UART_ENABLE_IT(&_huart, UART_IT_RXNE);

    _ready = true;
}

void HardwareSerial::end()
{
    if (!_ready) return;
    __HAL_UART_DISABLE_IT(&_huart, UART_IT_RXNE);
    if (_inst == USART2) HAL_NVIC_DisableIRQ(USART2_LPUART2_IRQn);
    HAL_UART_DeInit(&_huart);
    _ready = false;
}

size_t HardwareSerial::write(uint8_t c)
{
    if (!_ready) return 0;
    HAL_UART_Transmit(&_huart, &c, 1, HAL_MAX_DELAY);
    return 1;
}

void HardwareSerial::flush()
{
    if (!_ready) return;
    while (__HAL_UART_GET_FLAG(&_huart, UART_FLAG_TC) == RESET) { }
}

int HardwareSerial::available()
{
    return (int)((RX_BUF + _rx_head - _rx_tail) % RX_BUF);
}

int HardwareSerial::read()
{
    if (_rx_head == _rx_tail) return -1;
    uint8_t c = _rxbuf[_rx_tail];
    _rx_tail = (uint16_t)((_rx_tail + 1) % RX_BUF);
    return c;
}

int HardwareSerial::peek()
{
    if (_rx_head == _rx_tail) return -1;
    return _rxbuf[_rx_tail];
}

void HardwareSerial::_handleIRQ()
{
    /* RXNE path — grab the byte and stash it in the ring buffer. */
    if (__HAL_UART_GET_FLAG(&_huart, UART_FLAG_RXNE) != RESET) {
        uint8_t c = (uint8_t)(_huart.Instance->RDR & 0xFF);
        uint16_t next = (uint16_t)((_rx_head + 1) % RX_BUF);
        if (next != _rx_tail) {   /* drop on overflow */
            _rxbuf[_rx_head] = c;
            _rx_head = next;
        }
    }
    /* Clear sticky error flags so the UART stays alive under noise. */
    uint32_t isr = _huart.Instance->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) {
        _huart.Instance->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF;
    }
}

extern "C" void USART2_LPUART2_IRQHandler(void)
{
    if (s_usart2_instance) s_usart2_instance->_handleIRQ();
}
