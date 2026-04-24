#pragma once
#include <stdint.h>
#include <stddef.h>
#include "wiring_constants.h"

class __FlashStringHelper;

class Print {
public:
    virtual size_t write(uint8_t) = 0;
    virtual size_t write(const uint8_t* buf, size_t size);

    size_t write(const char* str)
    {
        if (!str) return 0;
        size_t n = 0;
        while (*str) { n += write((uint8_t)*str++); }
        return n;
    }

    size_t print(const char* s)            { return write(s); }
    size_t print(const __FlashStringHelper* s) { return write(reinterpret_cast<const char*>(s)); }
    size_t print(char c)                   { return write((uint8_t)c); }
    size_t print(unsigned char n, int base = DEC) { return printNumber((unsigned long)n, base); }
    size_t print(int n, int base = DEC);
    size_t print(unsigned int n, int base = DEC)  { return printNumber((unsigned long)n, base); }
    size_t print(long n, int base = DEC);
    size_t print(unsigned long n, int base = DEC) { return printNumber(n, base); }
    size_t print(double v, int digits = 2);

    size_t println(void)                    { return write((uint8_t)'\r') + write((uint8_t)'\n'); }
    size_t println(const char* s)           { size_t n = print(s); return n + println(); }
    size_t println(const __FlashStringHelper* s) { size_t n = print(s); return n + println(); }
    size_t println(char c)                  { size_t n = print(c); return n + println(); }
    size_t println(unsigned char v, int base = DEC) { size_t n = print(v, base); return n + println(); }
    size_t println(int v, int base = DEC)           { size_t n = print(v, base); return n + println(); }
    size_t println(unsigned int v, int base = DEC)  { size_t n = print(v, base); return n + println(); }
    size_t println(long v, int base = DEC)          { size_t n = print(v, base); return n + println(); }
    size_t println(unsigned long v, int base = DEC) { size_t n = print(v, base); return n + println(); }
    size_t println(double v, int digits = 2)        { size_t n = print(v, digits); return n + println(); }

    virtual void flush() {}

protected:
    size_t printNumber(unsigned long n, uint8_t base);
};
