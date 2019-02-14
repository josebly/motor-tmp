#ifndef UTIL_H
#define UTIL_H

#include "stdint.h"
#include "stm32f446xx.h"

inline uint32_t get_clock() { return DWT->CYCCNT; }

#endif