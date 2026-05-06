#include "Print.h"

size_t Print::write(const uint8_t* buf, size_t size)
{
    size_t n = 0;
    while (size--) { n += write(*buf++); }
    return n;
}

size_t Print::printNumber(unsigned long n, uint8_t base)
{
    char buf[8 * sizeof(long) + 1];
    char* str = &buf[sizeof(buf) - 1];
    *str = '\0';
    if (base < 2) base = 10;
    do {
        unsigned long m = n;
        n /= base;
        char c = (char)(m - base * n);
        *--str = (c < 10) ? (char)(c + '0') : (char)(c + 'A' - 10);
    } while (n);
    return write(str);
}

size_t Print::print(int n, int base)
{
    if (base == 10 && n < 0) {
        size_t t = write((uint8_t)'-');
        return t + printNumber((unsigned long)(-n), 10);
    }
    return printNumber((unsigned long)n, (uint8_t)base);
}

size_t Print::print(long n, int base)
{
    if (base == 10 && n < 0) {
        size_t t = write((uint8_t)'-');
        return t + printNumber((unsigned long)(-n), 10);
    }
    return printNumber((unsigned long)n, (uint8_t)base);
}

size_t Print::print(double v, int digits)
{
    size_t n = 0;
    if (v != v) return write("nan");
    if (v < 0.0) { n += write((uint8_t)'-'); v = -v; }
    double rounding = 0.5;
    for (int i = 0; i < digits; ++i) rounding /= 10.0;
    v += rounding;
    unsigned long whole = (unsigned long)v;
    double remainder = v - (double)whole;
    n += printNumber(whole, 10);
    if (digits > 0) n += write((uint8_t)'.');
    while (digits-- > 0) {
        remainder *= 10.0;
        unsigned int digit = (unsigned int)remainder;
        n += write((uint8_t)('0' + digit));
        remainder -= digit;
    }
    return n;
}
