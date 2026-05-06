#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define LOW  0
#define HIGH 1

#define INPUT             0x0
#define OUTPUT            0x1
#define INPUT_PULLUP      0x2
#define INPUT_PULLDOWN    0x3
#define OUTPUT_OPEN_DRAIN 0x4
#define INPUT_ANALOG      0x5

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE  1
#define FALLING 2
#define RISING  3

#ifndef PI
#define PI        3.1415926535897932384626433832795
#endif
#ifndef HALF_PI
#define HALF_PI   1.5707963267948966192313216916398
#endif
#ifndef TWO_PI
#define TWO_PI    6.283185307179586476925286766559
#endif

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#ifdef __cplusplus
}
#endif
