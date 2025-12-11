#ifndef GC9A01A_H
#define GC9A01A_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

#define GC9A01A_TFTWIDTH  240
#define GC9A01A_TFTHEIGHT 240

// --- General Colors ---
#define GC9A01A_BLACK   0x0000
#define GC9A01A_WHITE   0xFFFF

// --- Minion Eye Specific Colors (RGB565) ---
#define MINION_YELLOW    0xFFE0 
#define EYE_WHITE        0xFFFF
#define PUPIL_BLACK      0x0000
// --- Textured Iris Colors ---
#define IRIS_DARK_BROWN  0x5140 
#define IRIS_MAIN_BROWN  0x8200 
#define IRIS_LIGHT_BROWN 0xA302 

// --- Hardware Config ---
#define SPI_PORT spi0
#define PIN_SCK  18
#define PIN_MOSI 19

typedef struct {
    spi_inst_t *spi_instance;
    uint8_t cs_pin;
    uint8_t dc_pin;
    int8_t rst_pin;
    int dma_tx_channel;
    dma_channel_config dma_config;
} gc9a01a_t;

// --- Function Prototypes ---
void gc9a01a_init_struct(gc9a01a_t *tft, uint8_t cs_pin, uint8_t dc_pin, int8_t rst_pin);
void gc9a01a_begin(gc9a01a_t *tft);
void gc9a01a_update(gc9a01a_t *tft);
void gc9a01a_wait_for_update(gc9a01a_t *tft);
void gc9a01a_clear_framebuffer(uint16_t color);

// UPDATED RENDER FUNCTION PROTOTYPE
void gc9a01a_render_minion_frame(gc9a01a_t *tft, int16_t pupil_x, int16_t pupil_y, float upper_lid_h, float lower_lid_h);

#endif