#ifndef GC9A01A_H
#define GC9A01A_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // For memset

// --- Pico SDK Includes ---
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

// --- Display Dimensions ---
#define GC9A01A_TFTWIDTH  240
#define GC9A01A_TFTHEIGHT 240

// --- Color Definitions (16-bit RGB565) ---
#define GC9A01A_BLACK   0x0000
#define GC9A01A_WHITE   0xFFFF
#define GC9A01A_RED     0xF800
#define GC9A01A_GREEN   0x07E0
#define GC9A01A_BLUE    0x001F
#define GC9A01A_CYAN    0x07FF
#define GC9A01A_MAGENTA 0xF81F
#define GC9A01A_YELLOW  0xFFE0
#define GC9A01A_ORANGE  0xFC00

// --- Minion Eye Colors ---
#define MINION_YELLOW 0xFF40
#define IRIS_BROWN    0x8000
#define PUPIL_BLACK   0x0000
#define EYE_WHITE     0xFFFF
#define BORDER_BLACK  0x0000

// --- Hardware Config ---
#define SPI_PORT spi0
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_MISO 16 // Not used, but good to define if needed later

// --- Driver Struct ---
typedef struct {
    spi_inst_t *spi_instance;
    uint8_t cs_pin;
    uint8_t dc_pin;
    int8_t rst_pin;
    
    // DMA specific fields
    int dma_tx_channel;
    dma_channel_config dma_config;
} gc9a01a_t;

// --- Function Prototypes ---

// Core Setup
void gc9a01a_init_struct(gc9a01a_t *tft, uint8_t cs_pin, uint8_t dc_pin, int8_t rst_pin);
void gc9a01a_begin(gc9a01a_t *tft);

// Buffer Management & DMA
void gc9a01a_update(gc9a01a_t *tft);      // Sends the buffer to screen (DMA)
void gc9a01a_wait_for_update(gc9a01a_t *tft); // Blocks until DMA is finished
void gc9a01a_clear_framebuffer(uint16_t color);

// Graphics (Write to Buffer)
void gc9a01a_draw_pixel(int16_t x, int16_t y, uint16_t color);
void gc9a01a_fill_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

// Minion Animation
// void gc9a01a_draw_minion_eye_frame(gc9a01a_t *tft, float angle_deg);

// ... previous includes ...
#define MINION_YELLOW 0xFFE0 // Brighter Yellow
#define IRIS_BROWN    0x8800 // Reddish Brown
#define PUPIL_BLACK   0x0000
#define EYE_WHITE     0xFFFF
// ...
void gc9a01a_render_minion_frame(gc9a01a_t *tft, int16_t pupil_x, int16_t pupil_y, int16_t eyelid_height);
#endif // GC9A01A_H