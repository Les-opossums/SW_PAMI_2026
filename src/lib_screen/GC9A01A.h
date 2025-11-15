#ifndef GC9A01A_H
#define GC9A01A_H

#include <stdint.h>
#include <stdbool.h>

// --- Pico SDK specific includes (Conceptual, requires actual SDK setup) ---
// #include "pico/stdlib.h"
// #include "hardware/spi.h"
// #include "hardware/gpio.h"

// --- Display Dimensions ---
#define GC9A01A_TFTWIDTH 240
#define GC9A01A_TFTHEIGHT 240

// --- GC9A01A Command Codes ---
#define GC9A01A_SWRESET 0x01
#define GC9A01A_SLPOUT 0x11
#define GC9A01A_INVOFF 0x20
#define GC9A01A_INVON 0x21
#define GC9A01A_DISPOFF 0x28
#define GC9A01A_DISPON 0x29
#define GC9A01A_CASET 0x2A
#define GC9A01A_RASET 0x2B
#define GC9A01A_RAMWR 0x2C
#define GC9A01A_MADCTL 0x36
#define GC9A01A_COLMOD 0x3A
#define GC9A01A_TEON 0x35
#define GC9A01A_INREGEN2 0xEF
#define GC9A01A_INREGEN1 0xFE
#define GC9A01A1_POWER2 0xC3
#define GC9A01A1_POWER3 0xC4
#define GC9A01A1_POWER4 0xC9
#define GC9A01A_FRAMERATE 0xE8
#define GC9A01A_GAMMA1 0xF0
#define GC9A01A_GAMMA2 0xF1
#define GC9A01A_GAMMA3 0xF2
#define GC9A01A_GAMMA4 0xF3

// --- MADCTL Options (Memory Access Control) ---
#define MADCTL_MY 0x80
#define MADCTL_MX 0x40
#define MADCTL_MV 0x20
#define MADCTL_BGR 0x08

// --- Color Definitions (16-bit RGB565) ---
#define GC9A01A_BLACK 0x0000
#define GC9A01A_WHITE 0xFFFF
#define GC9A01A_RED 0xF800
#define GC9A01A_GREEN 0x07E0
#define GC9A01A_BLUE 0x001F

// Define the SPI instance and Pinouts for Pico W
#define SPI_PORT spi0
#define PIN_SCK 18
#define PIN_MOSI 19
// MISO is not typically used for writing to a display, but defined for completeness
#define PIN_MISO 13

// --- C Equivalent of the Adafruit_GC9A01A Class ---
typedef struct {
    // Pico-specific fields
    spi_inst_t *spi_instance; // Pointer to the actual SPI peripheral (SPI0 or SPI1)

    // Original fields
    uint8_t cs_pin;
    uint8_t dc_pin;
    int8_t rst_pin; // -1 if unused

    uint16_t _width;
    uint16_t _height;
    uint8_t rotation;
} gc9a01a_t;
// --- Low-Level SPI/GPIO Functions (MUST be implemented with Pico SDK) ---
// These functions replace Adafruit_SPITFT's low-level communications.
void spi_init_device(gc9a01a_t *tft, uint32_t freq);
void spi_write_command(gc9a01a_t *tft, uint8_t command);
void spi_write_data(gc9a01a_t *tft, const uint8_t *data, size_t len);
void spi_write_data_byte(gc9a01a_t *tft, uint8_t data);
void spi_write_data_16bit(gc9a01a_t *tft, uint16_t data);
void delay_ms(uint32_t ms);

// --- Core Driver Functions ---

/**
 * @brief Initialize the GC9A01A structure and setup GPIO pins.
 * @param cs_pin Chip Select (CS) GPIO pin.
 * @param dc_pin Data/Command (DC) GPIO pin.
 * @param rst_pin Reset (RST) GPIO pin, or -1 if unused.
 */
void gc9a01a_init_struct(gc9a01a_t *tft, uint8_t cs_pin, uint8_t dc_pin, int8_t rst_pin);

/**
 * @brief Send initialization commands to the GC9A01A chip.
 * @param tft Pointer to the display structure.
 * @param freq Desired SPI clock frequency.
 */
void gc9a01a_begin(gc9a01a_t *tft, uint32_t freq);

/**
 * @brief Set the address window for subsequent memory write operations (RAMWR).
 * @param tft Pointer to the display structure.
 * @param x Start column address.
 * @param y Start row address.
 * @param w Width of the window.
 * @param h Height of the window.
 */
void gc9a01a_set_addr_window(gc9a01a_t *tft, uint16_t x, uint16_t y, uint16_t w, uint16_t h);

/**
 * @brief Set the display rotation.
 * @param tft Pointer to the display structure.
 * @param rotation The index for rotation, from 0-3 inclusive.
 */
void gc9a01a_set_rotation(gc9a01a_t *tft, uint8_t rotation);

/**
 * @brief Enable/Disable display color inversion.
 * @param tft Pointer to the display structure.
 * @param invert True to invert, False for normal color.
 */
void gc9a01a_invert_display(gc9a01a_t *tft, bool invert);

// --- Minion Eye Colors (RGB565) ---
#define MINION_YELLOW 0xFF40
#define IRIS_BROWN    0x8000
#define PUPIL_BLACK   0x0000
#define EYE_WHITE     0xFFFF
#define BORDER_BLACK  0x0000

// --- Minion Eye Geometry ---
#define CENTER_X 120
#define CENTER_Y 120
#define GOGGLE_R 120
#define EYE_R     90
#define IRIS_R    45
#define PUPIL_R   20
#define MOVE_LIMIT 35

// --- Graphics Functions (Implemented in gc9a01a.c) ---
void gc9a01a_draw_pixel(gc9a01a_t *tft, uint16_t x, uint16_t y, uint16_t color);
void gc9a01a_fill_circle(gc9a01a_t *tft, int16_t x0, int16_t y0, int16_t r, uint16_t color);
void gc9a01a_draw_minion_eye(gc9a01a_t *tft, int16_t pupil_cx, int16_t pupil_cy);
void gc9a01a_animate_eye(gc9a01a_t *tft);

#endif // GC9A01A_H