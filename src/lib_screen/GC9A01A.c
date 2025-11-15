#include "../PAMI_2026.h"
#include <stddef.h> // For size_t

// Default SPI Frequency (simplified from original complex defines)
#define SPI_DEFAULT_FREQ 40000000 

// --- Command Initialization Array (Stored in Flash on Pico) ---
// Note: The original code used PROGMEM and pgm_read_byte for AVR/Arduino.
// On Pico, this array is simply a global const array in RAM/FLASH.
// Format: Command, Count | Delay Flag (0x80), Data...
const uint8_t initcmd[] = {
    GC9A01A_INREGEN2, 0,
    0xEB, 1, 0x14,
    GC9A01A_INREGEN1, 0,
    GC9A01A_INREGEN2, 0,
    0xEB, 1, 0x14,
    0x84, 1, 0x40,
    0x85, 1, 0xFF,
    0x86, 1, 0xFF,
    0x87, 1, 0xFF,
    0x88, 1, 0x0A,
    0x89, 1, 0x21,
    0x8A, 1, 0x00,
    0x8B, 1, 0x80,
    0x8C, 1, 0x01,
    0x8D, 1, 0x01,
    0x8E, 1, 0xFF,
    0x8F, 1, 0xFF,
    0xB6, 2, 0x00, 0x00,
    GC9A01A_MADCTL, 1, MADCTL_MX | MADCTL_BGR,
    GC9A01A_COLMOD, 1, 0x05,
    0x90, 4, 0x08, 0x08, 0x08, 0x08,
    0xBD, 1, 0x06,
    0xBC, 1, 0x00,
    0xFF, 3, 0x60, 0x01, 0x04,
    GC9A01A1_POWER2, 1, 0x13,
    GC9A01A1_POWER3, 1, 0x13,
    GC9A01A1_POWER4, 1, 0x22,
    0xBE, 1, 0x11,
    0xE1, 2, 0x10, 0x0E,
    0xDF, 3, 0x21, 0x0c, 0x02,
    GC9A01A_GAMMA1, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
    GC9A01A_GAMMA2, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,
    GC9A01A_GAMMA3, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
    GC9A01A_GAMMA4, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,
    0xED, 2, 0x1B, 0x0B,
    0xAE, 1, 0x77,
    0xCD, 1, 0x63,
    GC9A01A_FRAMERATE, 1, 0x34,
    0x62, 12, 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70,
    0x63, 12, 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70,
    0x64, 7, 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07,
    0x66, 10, 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00,
    0x67, 10, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98,
    0x74, 7, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00,
    0x98, 2, 0x3e, 0x07,
    GC9A01A_TEON, 0,
    GC9A01A_INVON, 0,
    GC9A01A_SLPOUT, 0x80, // Exit sleep (with 150ms delay)
    GC9A01A_DISPON, 0x80, // Display on (with 150ms delay)
    0x00 // End of list
};


// --- Low-Level SPI/GPIO Functions (Pico SDK Implementation) ---

/**
 * @brief Initializes the SPI hardware and sets up all required GPIO pins.
 * @param tft Pointer to the display structure.
 * @param freq Desired SPI clock frequency (e.g., 40000000 for 40MHz).
 */
void spi_init_device(gc9a01a_t *tft, uint32_t freq) {
    // 1. Assign SPI instance
    tft->spi_instance = SPI_PORT;

    // 2. Initialize GPIO pins for SPI function
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // gpio_set_function(PIN_MISO, GPIO_FUNC_SPI); // Uncomment if MISO is needed

    // 3. Initialize the SPI peripheral
    uint32_t actual_freq = spi_init(tft->spi_instance, freq);
    printf("SPI Initialized at %d Hz\n", actual_freq); // Debug output

    // 4. Initialize display-specific control pins (CS, DC, RST)
    // CS Pin (Active Low)
    gpio_init(tft->cs_pin);
    gpio_set_dir(tft->cs_pin, GPIO_OUT);
    gpio_put(tft->cs_pin, 1); // Deselect chip initially

    // DC Pin (Data/Command - Low for Command, High for Data)
    gpio_init(tft->dc_pin);
    gpio_set_dir(tft->dc_pin, GPIO_OUT);

    // RST Pin (If available, setup for output)
    if (tft->rst_pin >= 0) {
        gpio_init(tft->rst_pin);
        gpio_set_dir(tft->rst_pin, GPIO_OUT);
        // Perform a hardware reset sequence (replaces the old Arduino code)
        gpio_put(tft->rst_pin, 0); // Active low reset
        delay_ms(10);
        gpio_put(tft->rst_pin, 1); // Release reset
        delay_ms(100);
    }
}

/**
 * @brief Performs a blocking write of a single command byte.
 */
void spi_write_command(gc9a01a_t *tft, uint8_t command) {
    // 1. Set DC low for COMMAND
    gpio_put(tft->dc_pin, 0);

    // 2. Select the chip (CS low)
    gpio_put(tft->cs_pin, 0);

    // 3. Write the command byte
    spi_write_blocking(tft->spi_instance, &command, 1);

    // 4. Deselect the chip (CS high)
    // NOTE: This is often left low until all data for the command is sent,
    // but some libraries lift it immediately. Keeping it low is safer for parameter writing.
    // We will manage CS manually in the command functions, but it's often done here for simplicity.
    // For now, keep it low for potential data write next.
    // gpio_put(tft->cs_pin, 1);
}

/**
 * @brief Performs a blocking write of data bytes following a command.
 */
void spi_write_data(gc9a01a_t *tft, const uint8_t *data, size_t len) {
    // NOTE: spi_write_command already set CS low.

    // 1. Set DC high for DATA
    gpio_put(tft->dc_pin, 1);

    // 2. Write the data buffer
    spi_write_blocking(tft->spi_instance, data, len);

    // 3. Deselect the chip (CS high) - Transaction complete
    gpio_put(tft->cs_pin, 1);
}

/**
 * @brief Performs a blocking write of a single 8-bit data byte.
 */
void spi_write_data_byte(gc9a01a_t *tft, uint8_t data) {
    // This is typically called only when CS is already low (within a transaction)
    // 1. Set DC high for DATA
    gpio_put(tft->dc_pin, 1);

    // 2. Write the data byte
    spi_write_blocking(tft->spi_instance, &data, 1);
}

/**
 * @brief Performs a blocking write of a single 16-bit word (often used for colors/addresses).
 * The GC9A01A is typically Big-Endian (MSB first).
 */
void spi_write_data_16bit(gc9a01a_t *tft, uint16_t data) {
    // This is typically called only when CS is already low (within a transaction)
    // 1. Set DC high for DATA
    gpio_put(tft->dc_pin, 1);

    // 2. Split the 16-bit word into two 8-bit bytes (Big-Endian: MSB, LSB)
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(data >> 8); // Most Significant Byte
    buffer[1] = (uint8_t)(data & 0xFF); // Least Significant Byte

    // 3. Write the two bytes
    spi_write_blocking(tft->spi_instance, buffer, 2);
}

/**
 * @brief Simple delay function using the Pico SDK.
 */
void delay_ms(uint32_t ms) {
    sleep_ms(ms);
}

// --- Helper Functions (Pico SDK SPI/GPIO implementation needed) ---
/*
 * NOTE: The four low-level functions must be implemented using the Pico SDK.
 * For example, in spi_write_command, you would set the DC pin LOW, CS pin LOW,
 * use spi_write_blocking(), then set CS pin HIGH.
 *
 * Example (Conceptual):
 * void spi_write_command(gc9a01a_t *tft, uint8_t command) {
 * gpio_put(tft->dc_pin, 0); // DC low for command
 * gpio_put(tft->cs_pin, 0); // CS low
 * spi_write_blocking(SPI_PORT, &command, 1);
 * gpio_put(tft->cs_pin, 1); // CS high
 * }
 *
 * void delay_ms(uint32_t ms) {
 * sleep_ms(ms);
 * }
 */

// --- Core Driver Implementations ---

void gc9a01a_init_struct(gc9a01a_t *tft, uint8_t cs_pin, uint8_t dc_pin, int8_t rst_pin) {
    tft->cs_pin = cs_pin;
    tft->dc_pin = dc_pin;
    tft->rst_pin = rst_pin;
    tft->_width = GC9A01A_TFTWIDTH;
    tft->_height = GC9A01A_TFTHEIGHT;
    tft->rotation = 0;
    // Pin initialization (MUST be done with Pico SDK gpio_init/gpio_set_dir)
}

/**
 * @brief Send a command with optional data.
 * This is the C equivalent of Adafruit_SPITFT::sendCommand(cmd, addr, numArgs).
 */
static void gc9a01a_send_command(gc9a01a_t *tft, uint8_t command, const uint8_t *data, uint8_t num_args) {
    spi_write_command(tft, command);
    if (num_args > 0) {
        spi_write_data(tft, data, num_args);
    }
}

void gc9a01a_begin(gc9a01a_t *tft, uint32_t freq) {
    if (!freq) {
        freq = SPI_DEFAULT_FREQ;
    }
    // Initialize SPI hardware (Pico SDK implementation needed)
    spi_init_device(tft, freq);

    // Hardware reset (if reset pin is available)
    if (tft->rst_pin >= 0) {
        // gpio_put(tft->rst_pin, 0); // Pico SDK implementation needed
        // delay_ms(10);
        // gpio_put(tft->rst_pin, 1);
        // delay_ms(100);
    } else {
        // Software reset
        spi_write_command(tft, GC9A01A_SWRESET);
        delay_ms(150);
    }

    // Process initialization sequence
    uint8_t cmd, x, num_args;
    size_t i = 0;
    while ((cmd = initcmd[i++]) > 0) {
        x = initcmd[i++];
        num_args = x & 0x7F;

        gc9a01a_send_command(tft, cmd, &initcmd[i], num_args);
        i += num_args;

        if (x & 0x80) { // Check for delay flag
            delay_ms(150);
        }
    }

    tft->_width = GC9A01A_TFTWIDTH;
    tft->_height = GC9A01A_TFTHEIGHT;
}

void gc9a01a_set_addr_window(gc9a01a_t *tft, uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) {
    uint16_t x2 = (x1 + w - 1);
    uint16_t y2 = (y1 + h - 1);

    spi_write_command(tft, GC9A01A_CASET); // Column address set
    // NOTE: SPI_WRITE16 is typically a macro that writes two bytes (MSB first)
    spi_write_data_16bit(tft, x1);
    spi_write_data_16bit(tft, x2);

    spi_write_command(tft, GC9A01A_RASET); // Row address set
    spi_write_data_16bit(tft, y1);
    spi_write_data_16bit(tft, y2);

    spi_write_command(tft, GC9A01A_RAMWR); // Write to RAM
}

void gc9a01a_set_rotation(gc9a01a_t *tft, uint8_t m) {
    uint8_t madctl;
    tft->rotation = m % 4; // can't be higher than 3

    switch (tft->rotation) {
    case 0:
        madctl = (MADCTL_MX | MADCTL_BGR);
        tft->_width = GC9A01A_TFTWIDTH;
        tft->_height = GC9A01A_TFTHEIGHT;
        break;
    case 1:
        madctl = (MADCTL_MV | MADCTL_BGR);
        tft->_width = GC9A01A_TFTHEIGHT;
        tft->_height = GC9A01A_TFTWIDTH;
        break;
    case 2:
        madctl = (MADCTL_MY | MADCTL_BGR);
        tft->_width = GC9A01A_TFTWIDTH;
        tft->_height = GC9A01A_TFTHEIGHT;
        break;
    case 3:
        madctl = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
        tft->_width = GC9A01A_TFTHEIGHT;
        tft->_height = GC9A01A_TFTWIDTH;
        break;
    default:
        return; // Should not happen
    }

    gc9a01a_send_command(tft, GC9A01A_MADCTL, &madctl, 1);
}

void gc9a01a_invert_display(gc9a01a_t *tft, bool invert) {
    spi_write_command(tft, invert ? GC9A01A_INVON : GC9A01A_INVOFF);
}


// =========================================================================
// GRAPHICS AND ANIMATION FUNCTIONS
// =========================================================================

void gc9a01a_draw_pixel(gc9a01a_t *tft, uint16_t x, uint16_t y, uint16_t color) {
    if ((x >= tft->_width) || (y >= tft->_height)) return;
    
    // 1. Set the 1x1 address window
    gc9a01a_set_addr_window(tft, x, y, 1, 1);

    // 2. Write the 16-bit color data (CS is low after RAMWR command)
    gpio_put(tft->dc_pin, 1); 
    spi_write_data_16bit(tft, color);
    
    // 3. End transaction
    gpio_put(tft->cs_pin, 1); 
}

// Optimized line drawing function for filling a circle row (internal use)
static void gc9a01a_draw_fast_hline(gc9a01a_t *tft, int16_t x, int16_t y, int16_t w, uint16_t color) {
    if (w <= 0 || y < 0 || y >= tft->_height || x >= tft->_width || x + w <= 0) return;

    int16_t x2 = x + w - 1;
    if (x < 0) { w += x; x = 0; }
    if (x2 >= tft->_width) { w = tft->_width - x; }

    if (w > 0) {
        gc9a01a_set_addr_window(tft, x, y, w, 1);
        gpio_put(tft->dc_pin, 1); // Data mode

        // Write pixel data in a block
        for (int16_t i = 0; i < w; i++) {
            spi_write_data_16bit(tft, color);
        }
        gpio_put(tft->cs_pin, 1); // End transaction
    }
}

void gc9a01a_fill_circle(gc9a01a_t *tft, int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    gc9a01a_draw_fast_hline(tft, x0 - r, y0, 2 * r + 1, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        gc9a01a_draw_fast_hline(tft, x0 - x, y0 + y, 2 * x + 1, color);
        gc9a01a_draw_fast_hline(tft, x0 - x, y0 - y, 2 * x + 1, color);
        gc9a01a_draw_fast_hline(tft, x0 - y, y0 + x, 2 * y + 1, color);
        gc9a01a_draw_fast_hline(tft, x0 - y, y0 - x, 2 * y + 1, color);
    }
}

void gc9a01a_draw_minion_eye(gc9a01a_t *tft, int16_t pupil_cx, int16_t pupil_cy) {
    // 1. Draw the surrounding Goggle/Minion skin (Yellow/Gold)
    gc9a01a_fill_circle(tft, CENTER_X, CENTER_Y, GOGGLE_R, MINION_YELLOW);
    
    // 2. Draw the Black Border Ring (Goggle Frame)
    gc9a01a_fill_circle(tft, CENTER_X, CENTER_Y, EYE_R + 5, BORDER_BLACK);
    
    // 3. Draw the White of the Eye
    gc9a01a_fill_circle(tft, CENTER_X, CENTER_Y, EYE_R, EYE_WHITE);
    
    // 4. Draw the Iris (Brown/Colored part)
    gc9a01a_fill_circle(tft, pupil_cx, pupil_cy, IRIS_R, IRIS_BROWN);
    
    // 5. Draw the Pupil (Black dot)
    gc9a01a_fill_circle(tft, pupil_cx, pupil_cy, PUPIL_R, PUPIL_BLACK);
}

// Function to calculate clamped pupil position
static void calculate_pupil_position(float angle_deg, int16_t *out_x, int16_t *out_y) {
    float angle_rad = angle_deg * M_PI / 180.0f;
    
    int16_t offset_x = (int16_t)(MOVE_LIMIT * cos(angle_rad));
    int16_t offset_y = (int16_t)(MOVE_LIMIT * sin(angle_rad));
    
    *out_x = CENTER_X + offset_x;
    *out_y = CENTER_Y + offset_y;
}

void gc9a01a_animate_eye(gc9a01a_t *tft) {
    static int16_t current_pupil_x = CENTER_X;
    static int16_t current_pupil_y = CENTER_Y;
    static float current_angle = 0.0f;

    // A. Redraw the previous iris/pupil block with White to clear it
    gc9a01a_fill_circle(tft, current_pupil_x, current_pupil_y, IRIS_R, EYE_WHITE);
        
    // B. Calculate New Position 
    current_angle += 5.0f; // Incremental movement
    if (current_angle >= 360.0f) {
        current_angle -= 360.0f;
    }
    calculate_pupil_position(current_angle, &current_pupil_x, &current_pupil_y);

    // C. Draw the new Iris and Pupil
    gc9a01a_fill_circle(tft, current_pupil_x, current_pupil_y, IRIS_R, IRIS_BROWN);
    gc9a01a_fill_circle(tft, current_pupil_x, current_pupil_y, PUPIL_R, PUPIL_BLACK);
    
    delay_ms(100); // Control animation speed (~33 FPS)
}