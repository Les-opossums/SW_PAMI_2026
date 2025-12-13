#ifndef GC9A01A_H
#define GC9A01A_H

#define GC9A01A_TFTWIDTH  240
#define GC9A01A_TFTHEIGHT 240

// --- Colors ---
#define GC9A01A_BLACK    0x0000
#define EYE_WHITE        0xFFFF
#define MINION_YELLOW    0xFFE0 
#define PUPIL_BLACK      0x0000
#define IRIS_DARK_BROWN  0x5140 
#define IRIS_MAIN_BROWN  0x8200 
#define IRIS_LIGHT_BROWN 0xA302 

// --- Hardware Config ---
#define SPI_PORT spi0
#define PIN_SCK  18
#define PIN_MOSI 19


// --- Internal Command definitions ---
#define GC9A01A_SWRESET 0x01
#define GC9A01A_CASET   0x2A
#define GC9A01A_RASET   0x2B
#define GC9A01A_RAMWR   0x2C


typedef struct {
    spi_inst_t *spi_instance;
    uint8_t cs_pin;
    uint8_t dc_pin;
    int8_t rst_pin;
    int dma_tx_channel;
    dma_channel_config dma_config;
} gc9a01a_t;

// Pointer to the buffer the CPU should currently draw into
extern uint16_t *current_draw_buffer;

// --- API ---
void gc9a01a_init_struct(gc9a01a_t *tft, uint8_t cs_pin, uint8_t dc_pin, int8_t rst_pin);
void gc9a01a_begin(gc9a01a_t *tft);

// Check if DMA is currently working (True = Don't touch screen yet)
bool gc9a01a_is_busy(void);

// Kick off background transfer (Returns immediately)
void gc9a01a_update_async(gc9a01a_t *tft);

// Render logic (now writes to a specific buffer)
void gc9a01a_render_minion_frame(uint16_t *buffer, int16_t pupil_x, int16_t pupil_y, float upper_lid_h, float lower_lid_h);

#endif