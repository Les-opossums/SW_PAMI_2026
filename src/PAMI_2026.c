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

    // --- State Variables ---
    
    // Position
    float cur_x = CENTER_X;
    float cur_y = CENTER_Y;
    float start_x = CENTER_X;
    float start_y = CENTER_Y;
    float target_x = CENTER_X;
    float target_y = CENTER_Y;

    // Movement Timing
    uint32_t move_start_time = 0;
    uint32_t move_duration = 0;
    uint32_t next_move_time = 0;
    bool is_moving = false;

    // Blink Timing
    float eyelid_h = 0;
    bool is_blinking = false;
    uint32_t blink_start_time = 0;
    uint32_t next_blink_time = to_ms_since_boot(get_absolute_time()) + 2000;
    
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // --- 1. Saccadic Movement Logic ---
        
        if (!is_moving) {
            // Waiting to move
            if (now >= next_move_time) {
                // Time to pick a new spot!
                is_moving = true;
                move_start_time = now;
                
                // Random duration for the move (speed): 80ms to 200ms (fast!)
                move_duration = 80 + (rand() % 120);
                
                // Pick random point in circle
                float angle = (float)(rand() % 360) * 0.01745f;
                float dist = (float)(rand() % MAX_PUPIL_DIST);
                
                start_x = cur_x;
                start_y = cur_y;
                target_x = CENTER_X + (cosf(angle) * dist);
                target_y = CENTER_Y + (sinf(angle) * dist);

                // Determine how long to stay there (Fixation time)
                // 300ms to 2500ms
                next_move_time = now + move_duration + 300 + (rand() % 2200);
            }
        } else {
            // Currently Moving
            float elapsed = (float)(now - move_start_time);
            if (elapsed >= move_duration) {
                // Arrived
                cur_x = target_x;
                cur_y = target_y;
                is_moving = false;
            } else {
                // Interpolate
                float progress = elapsed / (float)move_duration;
                float val = ease_out_cubic(progress);
                cur_x = start_x + (target_x - start_x) * val;
                cur_y = start_y + (target_y - start_y) * val;
            }
        }

        // --- 2. Blinking Logic ---
        
        if (!is_blinking) {
            if (now >= next_blink_time) {
                is_blinking = true;
                blink_start_time = now;
            }
        } else {
            // Blink Animation (200ms total)
            uint32_t blink_dur = 200;
            uint32_t t = now - blink_start_time;
            
            if (t >= blink_dur) {
                is_blinking = false;
                eyelid_h = 0;
                // Random time until next blink (1s to 6s)
                next_blink_time = now + 1000 + (rand() % 5000);
            } else {
                // Calculate Eyelid Height
                // First half: Close. Second half: Open.
                if (t < (blink_dur / 2)) {
                    // Closing
                    float p = (float)t / (blink_dur / 2.0f);
                    eyelid_h = p * BLINK_CLOSE_Y;
                } else {
                    // Opening
                    float p = (float)(t - (blink_dur / 2)) / (blink_dur / 2.0f);
                    eyelid_h = BLINK_CLOSE_Y * (1.0f - p);
                }
            }
        }

        // --- 3. Render & Update ---

        // Wait for previous frame to finish
        gc9a01a_wait_for_update(&tft);

        // Draw the frame to RAM
        gc9a01a_render_minion_frame(&tft, (int16_t)cur_x, (int16_t)cur_y, (int16_t)eyelid_h);

        // Push to screen via DMA
        gc9a01a_update(&tft);
        
        // No explicit delay needed; loop runs as fast as DMA/Draw allows.
        // Approx 30-50 FPS depending on SPI speed.
    }
    
    return 0;
}

void Init_All(void)
{
    init_motors();
    Init_Asserv();
}

