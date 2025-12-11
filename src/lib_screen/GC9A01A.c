#include "GC9A01A.h"
#include <math.h>

// --- Framebuffer ---
// 240x240 pixels * 2 bytes = 115,200 bytes
static uint16_t frame_buffer[GC9A01A_TFTWIDTH * GC9A01A_TFTHEIGHT];

// --- Internal Command definitions ---
#define GC9A01A_SWRESET 0x01
#define GC9A01A_SLPOUT  0x11
#define GC9A01A_DISPON  0x29
#define GC9A01A_CASET   0x2A
#define GC9A01A_RASET   0x2B
#define GC9A01A_RAMWR   0x2C
#define GC9A01A_MADCTL  0x36
#define GC9A01A_COLMOD  0x3A

const uint8_t initcmd[] = {
    0xEF, 0, 0xEB, 1, 0x14, 0xFE, 0, 0xEF, 0, 0xEB, 1, 0x14, 0x84, 1, 0x40, 0x85, 1, 0xFF, 0x86, 1, 0xFF,
    0x87, 1, 0xFF, 0x88, 1, 0x0A, 0x89, 1, 0x21, 0x8A, 1, 0x00, 0x8B, 1, 0x80, 0x8C, 1, 0x01, 0x8D, 1, 0x01,
    0x8E, 1, 0xFF, 0x8F, 1, 0xFF, 0xB6, 2, 0x00, 0x00, 0x36, 1, 0x48, 0x3A, 1, 0x05, 0x90, 4, 0x08, 0x08, 0x08, 0x08,
    0xBD, 1, 0x06, 0xBC, 1, 0x00, 0xFF, 3, 0x60, 0x01, 0x04, 0xC3, 1, 0x13, 0xC4, 1, 0x13, 0xC9, 1, 0x22,
    0xBE, 1, 0x11, 0xE1, 2, 0x10, 0x0E, 0xDF, 3, 0x21, 0x0c, 0x02, 0xF0, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
    0xF1, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F, 0xF2, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
    0xF3, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F, 0xED, 2, 0x1B, 0x0B, 0xAE, 1, 0x77, 0xCD, 1, 0x63,
    0xE8, 1, 0x34, 0x62, 12, 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70,
    0x63, 12, 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70,
    0x64, 7, 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07, 0x66, 10, 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00,
    0x67, 10, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98, 0x74, 7, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00,
    0x98, 2, 0x3e, 0x07, 0x35, 0, 0x21, 0, 0x11, 0x80, 0x29, 0x80, 0x00
};

// --- Low Level Internal Helpers ---

static void spi_write_cmd(gc9a01a_t *tft, uint8_t cmd) {
    gpio_put(tft->dc_pin, 0); // Command
    gpio_put(tft->cs_pin, 0);
    spi_write_blocking(tft->spi_instance, &cmd, 1);
    gpio_put(tft->cs_pin, 1);
}

static void spi_write_data_buf(gc9a01a_t *tft, const uint8_t *data, size_t len) {
    gpio_put(tft->dc_pin, 1); // Data
    gpio_put(tft->cs_pin, 0);
    spi_write_blocking(tft->spi_instance, data, len);
    gpio_put(tft->cs_pin, 1);
}

void gc9a01a_init_struct(gc9a01a_t *tft, uint8_t cs_pin, uint8_t dc_pin, int8_t rst_pin) {
    tft->cs_pin = cs_pin;
    tft->dc_pin = dc_pin;
    tft->rst_pin = rst_pin;
    tft->spi_instance = SPI_PORT;
}

// --- Initialization ---

void gc9a01a_begin(gc9a01a_t *tft) {
    // 1. Initialize SPI
    spi_init(tft->spi_instance, 62500000); // 62.5MHz
    
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    gpio_init(tft->cs_pin);
    gpio_set_dir(tft->cs_pin, GPIO_OUT);
    gpio_put(tft->cs_pin, 1);

    gpio_init(tft->dc_pin);
    gpio_set_dir(tft->dc_pin, GPIO_OUT);
    gpio_put(tft->dc_pin, 1);

    if (tft->rst_pin >= 0) {
        gpio_init(tft->rst_pin);
        gpio_set_dir(tft->rst_pin, GPIO_OUT);
        gpio_put(tft->rst_pin, 1);
        sleep_ms(100);
        gpio_put(tft->rst_pin, 0);
        sleep_ms(100);
        gpio_put(tft->rst_pin, 1);
        sleep_ms(200);
    } else {
        spi_write_cmd(tft, GC9A01A_SWRESET);
        sleep_ms(150);
    }

    // 2. Initialize DMA for 8-BIT TRANSFERS
    tft->dma_tx_channel = dma_claim_unused_channel(true);
    tft->dma_config = dma_channel_get_default_config(tft->dma_tx_channel);
    
    // FIX: Send 8 bits at a time. This matches the standard SPI data width.
    channel_config_set_transfer_data_size(&tft->dma_config, DMA_SIZE_8); 
    channel_config_set_dreq(&tft->dma_config, spi_get_dreq(tft->spi_instance, true));
    
    // FIX: Do NOT use hardware bswap. We will swap in software.
    // channel_config_set_bswap(&tft->dma_config, true); 

    // 3. Send Init Commands
    uint8_t cmd, x, num_args;
    size_t i = 0;
    while ((cmd = initcmd[i++]) > 0) {
        x = initcmd[i++];
        num_args = x & 0x7F;
        spi_write_cmd(tft, cmd);
        if (num_args > 0) {
            spi_write_data_buf(tft, &initcmd[i], num_args);
            i += num_args;
        }
        if (x & 0x80) sleep_ms(150);
    }
    
    gc9a01a_clear_framebuffer(GC9A01A_BLACK);
}

// --- Buffer & DMA Functions ---

void gc9a01a_wait_for_update(gc9a01a_t *tft) {
    dma_channel_wait_for_finish_blocking(tft->dma_tx_channel);
    gpio_put(tft->cs_pin, 1);
}

void gc9a01a_update(gc9a01a_t *tft) {
    gc9a01a_wait_for_update(tft);

    // Set Address Window
    uint8_t caset[] = {0, 0, 0, 239};
    uint8_t raset[] = {0, 0, 0, 239};
    
    spi_write_cmd(tft, GC9A01A_CASET);
    spi_write_data_buf(tft, caset, 4);
    
    spi_write_cmd(tft, GC9A01A_RASET);
    spi_write_data_buf(tft, raset, 4);

    spi_write_cmd(tft, GC9A01A_RAMWR);
    
    gpio_put(tft->dc_pin, 1); 
    gpio_put(tft->cs_pin, 0); 
    
    dma_channel_configure(
        tft->dma_tx_channel,
        &tft->dma_config,
        &spi_get_hw(tft->spi_instance)->dr, 
        (uint8_t*)frame_buffer,              // Cast to uint8_t pointer
        GC9A01A_TFTWIDTH * GC9A01A_TFTHEIGHT * 2, // FIX: Send Total Bytes (Pixels * 2)
        true
    );
}

void gc9a01a_clear_framebuffer(uint16_t color) {
    // Swap bytes for storage so they send correctly via 8-bit DMA
    uint16_t swapped_color = __builtin_bswap16(color);
    for (uint32_t i = 0; i < GC9A01A_TFTWIDTH * GC9A01A_TFTHEIGHT; i++) {
        frame_buffer[i] = swapped_color;
    }
}

// --- Graphics Functions ---

void gc9a01a_draw_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= GC9A01A_TFTWIDTH || y < 0 || y >= GC9A01A_TFTHEIGHT) return;
    // FIX: Swap bytes here. RP2040 is Little Endian, Screen is Big Endian.
    frame_buffer[y * GC9A01A_TFTWIDTH + x] = __builtin_bswap16(color);
}

// Optimized Horizontal Line
static void draw_fast_hline_buf(int16_t x, int16_t y, int16_t w, uint16_t color) {
    if (y < 0 || y >= GC9A01A_TFTHEIGHT) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > GC9A01A_TFTWIDTH) { w = GC9A01A_TFTWIDTH - x; }
    if (w <= 0) return;

    // Pre-swap the color once
    uint16_t swapped_color = __builtin_bswap16(color);
    uint32_t offset = y * GC9A01A_TFTWIDTH + x;
    
    for (int i = 0; i < w; i++) {
        frame_buffer[offset + i] = swapped_color;
    }
}

void gc9a01a_fill_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    draw_fast_hline_buf(x0 - r, y0, 2 * r + 1, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        draw_fast_hline_buf(x0 - x, y0 + y, 2 * x + 1, color);
        draw_fast_hline_buf(x0 - x, y0 - y, 2 * x + 1, color);
        draw_fast_hline_buf(x0 - y, y0 + x, 2 * y + 1, color);
        draw_fast_hline_buf(x0 - y, y0 - x, 2 * y + 1, color);
    }
}

// --- Animation ---
static void calculate_pupil_position(float angle_deg, int16_t *out_x, int16_t *out_y) {
    #define CENTER_X 120
    #define CENTER_Y 120
    #define MOVE_LIMIT 35
    
    float angle_rad = angle_deg * 3.14159f / 180.0f;
    int16_t offset_x = (int16_t)(MOVE_LIMIT * cosf(angle_rad));
    int16_t offset_y = (int16_t)(MOVE_LIMIT * sinf(angle_rad));
    *out_x = CENTER_X + offset_x;
    *out_y = CENTER_Y + offset_y;
}

void gc9a01a_draw_minion_eye_frame(gc9a01a_t *tft, float angle_deg) {
    int16_t px, py;
    calculate_pupil_position(angle_deg, &px, &py);

    gc9a01a_fill_circle(120, 120, 120, MINION_YELLOW); 
    gc9a01a_fill_circle(120, 120, 95, BORDER_BLACK); 
    gc9a01a_fill_circle(120, 120, 90, EYE_WHITE);
    gc9a01a_fill_circle(px, py, 45, IRIS_BROWN);
    gc9a01a_fill_circle(px, py, 20, PUPIL_BLACK);
}