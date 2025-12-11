#include "PAMI_2026.h"

// --- Configuration ---
#define CENTER_X 120
#define CENTER_Y 120
#define MAX_PUPIL_DIST 35  // How far the eye can look from center
#define BLINK_CLOSE_Y 110  // How far eyelids close (120 = fully closed)
// Pin configuration (Matches the table above)
#define LCD_CS_PIN 17
#define LCD_DC_PIN 16
#define LCD_RST_PIN 15 // Use -1 if you skip the reset pin

// --- Helper: Ease-Out Interpolation ---
// Makes movement look organic (fast start, slow stop)
float ease_out_cubic(float t) {
    return 1.0f - powf(1.0f - t, 3.0f);
}

int main() {
    // Initialize standard I/O (needed for printf debugging)
    stdio_init_all();

    // Create Driver Instance
    gc9a01a_t tft;
    gc9a01a_init_struct(&tft, LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN);
    gc9a01a_begin(&tft); // 0 uses the default SPI_DEFAULT_FREQ (40MHz)

    // 2. Initialize Minion Logic
    minion_eye_init(&tft);
    
    while (1) {
        minion_eye_loop();
    }
    
    return 0;
}

void Init_All(void)
{
    init_motors();
    Init_Asserv();
}

