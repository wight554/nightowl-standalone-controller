#pragma once

#include <stdint.h>

#include "pico/types.h"

void neopixel_init(uint pin);
void neopixel_set(uint8_t r, uint8_t g, uint8_t b);
