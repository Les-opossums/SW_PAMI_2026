/**
 * @file GC9A01A.c
 * @brief Implementation of High-Perf GC9A01A Driver.
 */

 #include "../PAMI_2026.h"

// --- Internal Commands ---
#define CMD_SWRESET 0x01
#define CMD_SLPOUT  0x11
#define CMD_DISPON  0x29
#define CMD_CASET   0x2A
#define CMD_RASET   0x2B
#define CMD_RAMWR   0x2C

// --- Double Buffering Storage (230kB) ---
// Allocated in RAM. RP2350 (Pico 2) has 520kB, so this is safe.
static uint16_t frame_buffer_1[GC9A01A_TFTWIDTH * GC9A01A_TFTHEIGHT];
static uint16_t frame_buffer_2[GC9A01A_TFTWIDTH * GC9A01A_TFTHEIGHT];

// Pointers exposed to user and internal DMA
uint16_t *gc9a01a_draw_buffer = frame_buffer_1;      
static uint16_t *send_buffer = frame_buffer_2; 

static volatile bool dma_busy = false;

// Global context for ISR (Interrupt Service Routine)
static int global_dma_chan = -1;
static int global_cs_pin = -1;

// --- Initialization Command Sequence ---
static const uint8_t initcmd[] = {
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

// --- Low Level SPI Helpers ---
static inline void spi_write_cmd(gc9a01a_t *tft, uint8_t cmd) {
    gpio_put(tft->dc_pin, 0); 
    gpio_put(tft->cs_pin, 0);
    spi_write_blocking(tft->spi_instance, &cmd, 1);
    gpio_put(tft->cs_pin, 1);
}

static inline void spi_write_data(gc9a01a_t *tft, const uint8_t *data, size_t len) {
    gpio_put(tft->dc_pin, 1); 
    gpio_put(tft->cs_pin, 0);
    spi_write_blocking(tft->spi_instance, data, len);
    gpio_put(tft->cs_pin, 1);
}

// --- Interrupt Handler ---
static void dma_irq_handler(void) {
    if (dma_channel_get_irq0_status(global_dma_chan)) {
        dma_channel_acknowledge_irq0(global_dma_chan);
        dma_busy = false;
        gpio_put(global_cs_pin, 1); // Deselect Chip
    }
}

// --- Public API Implementation ---

void gc9a01a_init(gc9a01a_t *tft, uint8_t cs_pin, uint8_t dc_pin, int8_t rst_pin) {
    tft->cs_pin = cs_pin;
    tft->dc_pin = dc_pin;
    tft->rst_pin = rst_pin;
    tft->spi_instance = GC9A01A_SPI_PORT;
    
    // Set globals for ISR
    global_cs_pin = cs_pin;
}

void gc9a01a_begin(gc9a01a_t *tft) {
    // 1. Init SPI & GPIO
    spi_init(tft->spi_instance, 62500000); // 62.5MHz
    gpio_set_function(GC9A01A_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(GC9A01A_PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(tft->cs_pin); gpio_set_dir(tft->cs_pin, GPIO_OUT); gpio_put(tft->cs_pin, 1);
    gpio_init(tft->dc_pin); gpio_set_dir(tft->dc_pin, GPIO_OUT); gpio_put(tft->dc_pin, 1);

    // 2. Hardware Reset
    if (tft->rst_pin >= 0) {
        gpio_init(tft->rst_pin); gpio_set_dir(tft->rst_pin, GPIO_OUT);
        gpio_put(tft->rst_pin, 1); sleep_ms(100);
        gpio_put(tft->rst_pin, 0); sleep_ms(100);
        gpio_put(tft->rst_pin, 1); sleep_ms(200);
    } else {
        spi_write_cmd(tft, CMD_SWRESET);
        sleep_ms(150);
    }

    // 3. DMA Configuration
    tft->dma_tx_channel = dma_claim_unused_channel(true);
    global_dma_chan = tft->dma_tx_channel;

    tft->dma_config = dma_channel_get_default_config(tft->dma_tx_channel);
    channel_config_set_transfer_data_size(&tft->dma_config, DMA_SIZE_8); // SPI expects bytes
    channel_config_set_dreq(&tft->dma_config, spi_get_dreq(tft->spi_instance, true));
    
    dma_channel_set_irq0_enabled(tft->dma_tx_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // 4. Send Init Sequence
    uint8_t cmd, x, num_args;
    size_t i = 0;
    while ((cmd = initcmd[i++]) > 0) {
        x = initcmd[i++];
        num_args = x & 0x7F;
        spi_write_cmd(tft, cmd);
        if (num_args > 0) {
            spi_write_data(tft, &initcmd[i], num_args);
            i += num_args;
        }
        if (x & 0x80) sleep_ms(150);
    }
}

bool gc9a01a_is_busy(void) {
    return dma_busy;
}

void gc9a01a_update_async(gc9a01a_t *tft) {
    // Safety check: wait if previous transfer is still running
    // (This handles the case where CPU renders faster than SPI sends)
    while(dma_busy) tight_loop_contents();

    // 1. Swap Pointers
    uint16_t *temp = send_buffer;
    send_buffer = gc9a01a_draw_buffer;
    gc9a01a_draw_buffer = temp;

    // 2. Set Window Address (Blocking, very fast <5us)
    uint8_t window[] = {0, 0, 0, 239};
    spi_write_cmd(tft, CMD_CASET); spi_write_data(tft, window, 4);
    spi_write_cmd(tft, CMD_RASET); spi_write_data(tft, window, 4);
    spi_write_cmd(tft, CMD_RAMWR);

    // 3. Start DMA
    gpio_put(tft->dc_pin, 1); // Data Mode
    gpio_put(tft->cs_pin, 0); // Select Chip
    
    dma_busy = true;

    dma_channel_configure(
        tft->dma_tx_channel,
        &tft->dma_config,
        &spi_get_hw(tft->spi_instance)->dr, // Dest: SPI FIFO
        (uint8_t*)send_buffer,              // Source: The buffer we just finished drawing
        GC9A01A_TFTWIDTH * GC9A01A_TFTHEIGHT * 2, // Count (in bytes)
        true // Start!
    );
}

// --- Graphics Primitives Implementation ---

void gc9a01a_fill_screen(uint16_t color) {
    uint16_t swapped = __builtin_bswap16(color);
    // Use 32-bit writes for speed
    uint32_t double_pixel = (swapped << 16) | swapped;
    uint32_t *buf32 = (uint32_t*)gc9a01a_draw_buffer;
    int count = (GC9A01A_TFTWIDTH * GC9A01A_TFTHEIGHT) / 2;
    while(count--) *buf32++ = double_pixel;
}

void gc9a01a_draw_pixel(uint16_t *buffer, int16_t x, int16_t y, uint16_t color) {
    if ((x < 0) || (x >= GC9A01A_TFTWIDTH) || (y < 0) || (y >= GC9A01A_TFTHEIGHT)) return;
    buffer[y * GC9A01A_TFTWIDTH + x] = __builtin_bswap16(color);
}

static inline void draw_fast_hline(uint16_t *buff, int16_t x, int16_t y, int16_t w, uint16_t color) {
    if (y < 0 || y >= GC9A01A_TFTHEIGHT) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > GC9A01A_TFTWIDTH) { w = GC9A01A_TFTWIDTH - x; }
    if (w <= 0) return;

    uint16_t swapped = __builtin_bswap16(color);
    uint32_t offset = y * GC9A01A_TFTWIDTH + x;
    while (w--) buff[offset++] = swapped;
}

void gc9a01a_fill_circle(uint16_t *buffer, int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    draw_fast_hline(buffer, x0 - r, y0, 2 * r + 1, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        draw_fast_hline(buffer, x0 - x, y0 + y, 2 * x + 1, color);
        draw_fast_hline(buffer, x0 - x, y0 - y, 2 * x + 1, color);
        draw_fast_hline(buffer, x0 - y, y0 + x, 2 * y + 1, color);
        draw_fast_hline(buffer, x0 - y, y0 - x, 2 * y + 1, color);
    }
}