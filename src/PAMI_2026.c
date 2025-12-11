#include "PAMI_2026.h"

// Pin configuration (Matches the table above)
#define LCD_CS_PIN 17
#define LCD_DC_PIN 16
#define LCD_RST_PIN 15 // Use -1 if you skip the reset pin

int main() {
    // Initialize standard I/O (needed for printf debugging)
    stdio_init_all();

    // Create Driver Instance
    gc9a01a_t tft;

    printf("Starting GC9A01A Minion Eye Animation...\n");

    // 1. Initialize the display structure with your chosen GPIO pin numbers
    gc9a01a_init_struct(&tft, LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN);

    // 2. Initialize the display hardware and run initialization sequence
    gc9a01a_begin(&tft); // 0 uses the default SPI_DEFAULT_FREQ (40MHz)
    float angle = 0.0f;
    
    printf("Starting animation loop.\n");
    
    // 5. Animation Loop
    while (true) {
        // --- STEP 1: Sync ---
        // Ensure the PREVIOUS frame is done sending before we touch the buffer.
        // The first time run, this returns immediately.
        gc9a01a_wait_for_update(&tft);

        // --- STEP 2: Draw to RAM ---
        // The DMA is idle now, so we can safely modify the buffer.
        // We redraw the WHOLE eye every frame. Because it's just RAM writes, 
        // this is incredibly fast (microseconds/milliseconds).
        gc9a01a_draw_minion_eye_frame(&tft, angle);

        // --- STEP 3: Push to Screen ---
        // Trigger the DMA. This function sets up the transfer and returns INSTANTLY.
        gc9a01a_update(&tft);

        // --- STEP 4: CPU is Free! ---
        // While the screen is physically updating (taking ~30-40ms), 
        // the CPU is running down here.
        
        angle += 5.0f;
        if (angle >= 360.0f) angle -= 360.0f;

        // You can add a small sleep to control framerate if it's TOO fast,
        // or do other complex math here.
        sleep_ms(10);
    }
    
    return 0;
}

void Init_All(void)
{
    init_motors();
    Init_Asserv();
}

