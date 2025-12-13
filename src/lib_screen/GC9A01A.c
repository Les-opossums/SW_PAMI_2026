#include "../PAMI_2026.h"

// --- Double Buffering (230kB RAM usage - OK for Pico 2) ---
static uint16_t frame_buffer_1[GC9A01A_TFTWIDTH * GC9A01A_TFTHEIGHT];
static uint16_t frame_buffer_2[GC9A01A_TFTWIDTH * GC9A01A_TFTHEIGHT];

uint16_t *current_draw_buffer = frame_buffer_1;      
static uint16_t *current_send_buffer = frame_buffer_2; 

static volatile bool dma_busy = false;

// Global references for the Interrupt Handler
static int global_dma_channel = -1;
static int global_cs_pin = -1;

// --- Interrupt Handler (Runs when DMA finishes) ---
static void gc9a01a_dma_irq_handler(void) {
    if (dma_channel_get_irq0_status(global_dma_channel)) {
        dma_channel_acknowledge_irq0(global_dma_channel);
        dma_busy = false;
        gpio_put(global_cs_pin, 1); // Release CS
    }
}

// --- Init Commands (Condensed) ---
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

static void spi_write_cmd(gc9a01a_t *tft, uint8_t cmd) {
    gpio_put(tft->dc_pin, 0); gpio_put(tft->cs_pin, 0);
    spi_write_blocking(tft->spi_instance, &cmd, 1);
    gpio_put(tft->cs_pin, 1);
}
static void spi_write_data_buf(gc9a01a_t *tft, const uint8_t *data, size_t len) {
    gpio_put(tft->dc_pin, 1); gpio_put(tft->cs_pin, 0);
    spi_write_blocking(tft->spi_instance, data, len);
    gpio_put(tft->cs_pin, 1);
}

void gc9a01a_init_struct(gc9a01a_t *tft, uint8_t cs_pin, uint8_t dc_pin, int8_t rst_pin) {
    tft->cs_pin = cs_pin; tft->dc_pin = dc_pin; tft->rst_pin = rst_pin;
    tft->spi_instance = SPI_PORT;
    global_cs_pin = cs_pin;
}

void gc9a01a_begin(gc9a01a_t *tft) {
    spi_init(tft->spi_instance, 62500000); 
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI); gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(tft->cs_pin); gpio_set_dir(tft->cs_pin, GPIO_OUT); gpio_put(tft->cs_pin, 1);
    gpio_init(tft->dc_pin); gpio_set_dir(tft->dc_pin, GPIO_OUT); gpio_put(tft->dc_pin, 1);

    if (tft->rst_pin >= 0) {
        gpio_init(tft->rst_pin); gpio_set_dir(tft->rst_pin, GPIO_OUT);
        gpio_put(tft->rst_pin, 1); sleep_ms(100); gpio_put(tft->rst_pin, 0);
        sleep_ms(100); gpio_put(tft->rst_pin, 1); sleep_ms(200);
    } else {
        spi_write_cmd(tft, GC9A01A_SWRESET); sleep_ms(150);
    }

    // DMA Setup with Interrupts
    tft->dma_tx_channel = dma_claim_unused_channel(true);
    global_dma_channel = tft->dma_tx_channel;

    tft->dma_config = dma_channel_get_default_config(tft->dma_tx_channel);
    channel_config_set_transfer_data_size(&tft->dma_config, DMA_SIZE_8); 
    channel_config_set_dreq(&tft->dma_config, spi_get_dreq(tft->spi_instance, true));

    // Enable DMA IRQ 0
    dma_channel_set_irq0_enabled(tft->dma_tx_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, gc9a01a_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Send Init
    uint8_t cmd, x, num_args;
    size_t i = 0;
    while ((cmd = initcmd[i++]) > 0) {
        x = initcmd[i++]; num_args = x & 0x7F;
        spi_write_cmd(tft, cmd);
        if (num_args > 0) { spi_write_data_buf(tft, &initcmd[i], num_args); i += num_args; }
        if (x & 0x80) sleep_ms(150);
    }
}

// ---------------- NON-BLOCKING API ----------------

bool gc9a01a_is_busy(void) {
    return dma_busy;
}

void gc9a01a_update_async(gc9a01a_t *tft) {
    // 1. Swap Buffers
    // The buffer we were drawing to now becomes the send buffer
    uint16_t *temp = current_send_buffer;
    current_send_buffer = current_draw_buffer;
    current_draw_buffer = temp;

    // 2. Set Window (Blocking SPI - takes <5us, negligible)
    uint8_t window[] = {0, 0, 0, 239};
    spi_write_cmd(tft, 0x2A); spi_write_data_buf(tft, window, 4); // CASET
    spi_write_cmd(tft, 0x2B); spi_write_data_buf(tft, window, 4); // RASET
    spi_write_cmd(tft, 0x2C); // RAMWR
    
    // 3. Start DMA Transfer
    gpio_put(tft->dc_pin, 1); 
    gpio_put(tft->cs_pin, 0); // CS Low
    
    dma_busy = true;
    
    // Fire and forget!
    dma_channel_configure(tft->dma_tx_channel, &tft->dma_config,
        &spi_get_hw(tft->spi_instance)->dr, (uint8_t*)current_send_buffer, 
        GC9A01A_TFTWIDTH * GC9A01A_TFTHEIGHT * 2, true);
}

// --- Render Helper ---
static inline void draw_fast_hline_buf(uint16_t *buff, int16_t x, int16_t y, int16_t w, uint16_t color) {
    if (y < 0 || y >= GC9A01A_TFTHEIGHT) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > GC9A01A_TFTWIDTH) { w = GC9A01A_TFTWIDTH - x; }
    if (w <= 0) return;
    uint16_t swapped = __builtin_bswap16(color);
    uint32_t offset = y * GC9A01A_TFTWIDTH + x;
    while(w--) buff[offset++] = swapped;
}

static void fill_circle(uint16_t *buff, int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r, ddF_x = 1, ddF_y = -2 * r, x = 0, y = r;
    draw_fast_hline_buf(buff, x0 - r, y0, 2 * r + 1, color);
    while (x < y) {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++; ddF_x += 2; f += ddF_x;
        draw_fast_hline_buf(buff, x0 - x, y0 + y, 2 * x + 1, color);
        draw_fast_hline_buf(buff, x0 - x, y0 - y, 2 * x + 1, color);
        draw_fast_hline_buf(buff, x0 - y, y0 + x, 2 * y + 1, color);
        draw_fast_hline_buf(buff, x0 - y, y0 - x, 2 * y + 1, color);
    }
}

void gc9a01a_render_minion_frame(uint16_t *buffer, int16_t pupil_x, int16_t pupil_y, float upper_lid_h, float lower_lid_h) {
    // 1. Clear to Black
    uint16_t swapped_black = 0; 
    for (int i = 0; i < 240 * 240; i++) buffer[i] = swapped_black;

    // 2. Eye Components
    fill_circle(buffer, 120, 120, 120, EYE_WHITE);
    fill_circle(buffer, pupil_x, pupil_y, 62, IRIS_DARK_BROWN);
    fill_circle(buffer, pupil_x, pupil_y, 56, IRIS_MAIN_BROWN);
    fill_circle(buffer, pupil_x, pupil_y, 38, IRIS_LIGHT_BROWN);
    fill_circle(buffer, pupil_x, pupil_y, 26, PUPIL_BLACK);
    fill_circle(buffer, pupil_x + 14, pupil_y - 14, 9, EYE_WHITE);

    // 3. Eyelids (Parabolic)
    uint16_t swapped_yellow = __builtin_bswap16(MINION_YELLOW);
    float curve_intensity = 25.0f; 
    float normalize_factor = 1.0f / (120.0f * 120.0f); 

    if (upper_lid_h < 5 && lower_lid_h < 5) return; 

    for (int y = 0; y < 240; y++) {
        for (int x = 0; x < 240; x++) {
            float dx = (float)(x - 120);
            float curve_offset = (dx * dx) * normalize_factor * curve_intensity;

            if (y < (upper_lid_h + curve_offset)) {
                 buffer[y * 240 + x] = swapped_yellow;
            }
            if (y > (240.0f - lower_lid_h - curve_offset)) {
                 buffer[y * 240 + x] = swapped_yellow;
            }
        }
    }
}