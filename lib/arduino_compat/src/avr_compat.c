/*
 * AVR libc extensions Arduino code reaches for. Adequate for MySensors' usage
 * (stringifying MyMessage payload values).
 */
#include "Arduino.h"
#include <stdio.h>
#include <string.h>

char* ultoa(unsigned long v, char* s, int base)
{
    if (base < 2 || base > 16) { s[0] = 0; return s; }
    char tmp[34];
    int i = 0;
    if (v == 0) tmp[i++] = '0';
    else while (v) { unsigned long r = v % base; tmp[i++] = (char)((r < 10) ? ('0'+r) : ('A'+r-10)); v /= base; }
    int j = 0;
    while (i) s[j++] = tmp[--i];
    s[j] = 0;
    return s;
}

char* ltoa(long v, char* s, int base)
{
    if (v < 0 && base == 10) { s[0] = '-'; ultoa((unsigned long)(-v), s+1, base); return s; }
    return ultoa((unsigned long)v, s, base);
}

char* utoa(unsigned int v, char* s, int base) { return ultoa((unsigned long)v, s, base); }
char* itoa(int v, char* s, int base)           { return ltoa((long)v, s, base); }

char* dtostrf(double v, signed char width, unsigned char prec, char* s)
{
    char fmt[16];
    snprintf(fmt, sizeof(fmt), "%%%d.%df", (int)width, (int)prec);
    sprintf(s, fmt, v);
    return s;
}
