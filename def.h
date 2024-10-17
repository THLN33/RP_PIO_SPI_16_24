
#pragma once

#include <stdio.h>
#include <pico.h>
#include <pico/time.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#define TFT_COLOR_1BITS     1
#define TFT_COLOR_4BITS     4
#define TFT_COLOR_12BITS    12
#define TFT_COLOR_16BITS    16
#define TFT_COLOR_18BITS    18
#define TFT_COLOR_24BITS    24

#define TFT_COLORS          TFT_COLOR_18BITS

#include "tft_pinout.h"

#define LOW         0
#define HIGH        1

#define PIO_SPI_FREQUENCY  62500000UL
#define SPI_FREQUENCY 2500000UL
#define TFT_SPI_MODE 0


#define DC_C        gpio_put(TFT_DC, LOW)
#define DC_D        gpio_put(TFT_DC, HIGH)
#define CS_L        gpio_put(TFT_CS, LOW)
#define CS_H        gpio_put(TFT_CS, HIGH)



#define delay(ms) sleep_ms(ms)


void ILI9488_Hardware_Init(void);
void ILI9488_HardReset(void);
void ILI9488_Init_Registers(void);

void ILI9488_Init(void);
void setRotation(uint8_t m);


void test_pio_16_24(void);