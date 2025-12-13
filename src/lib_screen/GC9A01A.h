/**
 * @file GC9A01A.h
 * @brief High-Performance, Non-Blocking DMA Driver for GC9A01A Circular Display (240x240).
 * * Features:
 * - Double Buffering (Requires ~230kB RAM, intended for RP2040/RP2350).
 * - DMA-based background transfer (0% CPU usage during transmission).
 * - Basic graphics primitives (Pixel, Rect, Circle).
 * * @author Refactored for Pico 2 W
 */

#ifndef GC9A01A_H
#define GC9A01A_H

// --- Configuration ---
#define GC9A01A_TFTWIDTH  240
#define GC9A01A_TFTHEIGHT 240

// --- Standard Colors (RGB565) ---
#define GC9A01A_BLACK   0x0000
#define GC9A01A_WHITE   0xFFFF
#define GC9A01A_RED     0xF800
#define GC9A01A_GREEN   0x07E0
#define GC9A01A_BLUE    0x001F
#define GC9A01A_CYAN    0x07FF
#define GC9A01A_MAGENTA 0xF81F
#define GC9A01A_YELLOW  0xFFE0
#define GC9A01A_ORANGE  0xFC00

// --- Hardware Pins Definition ---
// Modify these if your wiring changes
#define GC9A01A_SPI_PORT spi0
#define GC9A01A_PIN_SCK  18
#define GC9A01A_PIN_MOSI 19

/**
 * @brief Driver Context Struct
 */
typedef struct {
    spi_inst_t *spi_instance;
    uint8_t cs_pin;
    uint8_t dc_pin;
    int8_t rst_pin;         // Set to -1 if using Software Reset
    int dma_tx_channel;
    dma_channel_config dma_config;
} gc9a01a_t;

/**
 * @brief Pointer to the active drawing buffer.
 * * The application should ALWAYS write pixels to this buffer.
 * The driver automatically swaps this pointer when update_async is called.
 */
extern uint16_t *gc9a01a_draw_buffer;

// --- Initialization & Core API ---

/**
 * @brief Initialize the driver structure and GPIOs.
 * @param tft Pointer to the handle.
 * @param cs_pin Chip Select pin.
 * @param dc_pin Data/Command pin.
 * @param rst_pin Reset pin (-1 for unused/software reset).
 */
void gc9a01a_init(gc9a01a_t *tft, uint8_t cs_pin, uint8_t dc_pin, int8_t rst_pin);

/**
 * @brief Begin SPI communication and send initialization sequence to display.
 * @param tft Pointer to the handle.
 */
void gc9a01a_begin(gc9a01a_t *tft);

/**
 * @brief Check if the DMA is currently sending a frame.
 * @return true if busy (do not update yet), false if ready.
 */
bool gc9a01a_is_busy(void);

/**
 * @brief Swaps the buffers and starts the DMA transfer in the background.
 * * This function returns immediately. 
 * The 'gc9a01a_draw_buffer' pointer is updated to point to the new free buffer.
 * @param tft Pointer to the handle.
 */
void gc9a01a_update_async(gc9a01a_t *tft);

// --- Graphics Primitives ---

/**
 * @brief Fill the entire current draw buffer with a color.
 * @param color RGB565 color.
 */
void gc9a01a_fill_screen(uint16_t color);

/**
 * @brief Draw a single pixel into the buffer.
 * @param buffer Target buffer (usually gc9a01a_draw_buffer).
 * @param x X coordinate.
 * @param y Y coordinate.
 * @param color RGB565 color.
 */
void gc9a01a_draw_pixel(uint16_t *buffer, int16_t x, int16_t y, uint16_t color);

/**
 * @brief Fill a circle.
 * @param buffer Target buffer.
 * @param x0 Center X.
 * @param y0 Center Y.
 * @param r Radius.
 * @param color RGB565 color.
 */
void gc9a01a_fill_circle(uint16_t *buffer, int16_t x0, int16_t y0, int16_t r, uint16_t color);

#endif // GC9A01A_H