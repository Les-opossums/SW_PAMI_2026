#include "../PAMI_2026.h"

// --- Configuration Constants ---
#define CENTER_X 120
#define CENTER_Y 120
#define MAX_PUPIL_DIST 35 
#define MAX_LID_HEIGHT 115.0f

// --- Private State Variables ---
static gc9a01a_t *tft; // Reference to the driver

// Position State
static float cur_x = CENTER_X, cur_y = CENTER_Y;
static float start_x = CENTER_X, start_y = CENTER_Y;
static float target_x = CENTER_X, target_y = CENTER_Y;
static uint32_t move_start_time = 0;
static uint32_t move_duration = 0;
static uint32_t next_move_time = 0;
static bool is_moving = false;

// Emotion Definitions
typedef enum {
    EMOTION_NORMAL,    // Slight lids
    EMOTION_HAPPY,     // Wide open
    EMOTION_TIRED,     // Half closed
    EMOTION_SUSPICIOUS // Squinting
} EmotionState;

static EmotionState current_emotion = EMOTION_NORMAL;
static uint32_t next_emotion_time = 0;

// Lid State (Current actual height)
static float cur_lid_u = 20.0f;
static float cur_lid_l = 15.0f;

// Target lid heights for emotions (Upper, Lower)
static const float emotion_targets[4][2] = {
    {20.0f, 15.0f}, // NORMAL
    {0.0f,  0.0f},  // HAPPY
    {65.0f, 40.0f}, // TIRED
    {85.0f, 85.0f}  // SUSPICIOUS
};

// Blink State
static bool is_blinking = false;
static uint32_t blink_start_time = 0;
static uint32_t next_blink_time = 0;

// --- Helper Functions ---

static float smooth_move(float current, float target, float speed) {
    if (fabs(current - target) < speed) return target;
    return current + (target - current) * speed;
}

static float ease_out_cubic(float t) { 
    return 1.0f - powf(1.0f - t, 3.0f); 
}

// --- Public API ---

void minion_eye_init(gc9a01a_t *driver) {
    tft = driver;
    srand(to_us_since_boot(get_absolute_time())); // Seed RNG
    
    // Set initial timers
    uint32_t now = to_ms_since_boot(get_absolute_time());
    next_move_time = now + 1000;
    next_emotion_time = now + 3000;
    next_blink_time = now + 2000;
}

void minion_eye_loop(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // --- 1. Emotion State Machine ---
    if (now >= next_emotion_time) {
        int pick = rand() % 10;
        if (pick < 4) current_emotion = EMOTION_NORMAL;
        else if (pick < 6) current_emotion = EMOTION_HAPPY;
        else if (pick < 8) current_emotion = EMOTION_TIRED;
        else current_emotion = EMOTION_SUSPICIOUS;

        next_emotion_time = now + 3000 + (rand() % 5000);
    }

    // --- 2. Saccadic Eye Movement ---
    if (!is_moving) {
        if (now >= next_move_time) {
            is_moving = true; 
            move_start_time = now;
            move_duration = 80 + (rand() % 120);
            
            float angle = (float)(rand() % 360) * 0.01745f;
            float range_mod = (current_emotion == EMOTION_SUSPICIOUS || current_emotion == EMOTION_TIRED) ? 0.6f : 1.0f;
            float dist = (float)(rand() % MAX_PUPIL_DIST) * range_mod;
            
            start_x = cur_x; 
            start_y = cur_y;
            target_x = CENTER_X + (cosf(angle) * dist);
            target_y = CENTER_Y + (sinf(angle) * dist);
            
            next_move_time = now + move_duration + 300 + (rand() % 2000);
        }
    } else {
        float elapsed = (float)(now - move_start_time);
        if (elapsed >= move_duration) {
            cur_x = target_x; 
            cur_y = target_y; 
            is_moving = false;
        } else {
            float val = ease_out_cubic(elapsed / (float)move_duration);
            cur_x = start_x + (target_x - start_x) * val;
            cur_y = start_y + (target_y - start_y) * val;
        }
    }

    // --- 3. Lid Animation (Interpolation) ---
    cur_lid_u = smooth_move(cur_lid_u, emotion_targets[current_emotion][0], 0.05f);
    cur_lid_l = smooth_move(cur_lid_l, emotion_targets[current_emotion][1], 0.05f);

    // --- 4. Blinking Logic ---
    float final_lid_u = cur_lid_u;
    float final_lid_l = cur_lid_l;

    if (!is_blinking) {
        if (now >= next_blink_time) {
            is_blinking = true; 
            blink_start_time = now;
        }
    } else {
        uint32_t blink_dur = 180;
        uint32_t t = now - blink_start_time;
        if (t >= blink_dur) {
            is_blinking = false;
            next_blink_time = now + 800 + (rand() % 4000);
        } else {
            float blink_adder = 0;
            if (t < (blink_dur / 2)) {
                 blink_adder = ((float)t / (blink_dur / 2.0f)) * MAX_LID_HEIGHT;
            } else {
                 blink_adder = ((float)(blink_dur - t) / (blink_dur / 2.0f)) * MAX_LID_HEIGHT;
            }
            final_lid_u = fminf(MAX_LID_HEIGHT, cur_lid_u + blink_adder);
            final_lid_l = fminf(MAX_LID_HEIGHT, cur_lid_l + blink_adder);
        }
    }

    // --- 5. Render & Hardware Update ---
    
    // Ensure previous frame is fully sent via DMA before drawing new one
    gc9a01a_wait_for_update(tft);
    
    // Draw logic to RAM buffer
    gc9a01a_render_minion_frame(tft, (int16_t)cur_x, (int16_t)cur_y, final_lid_u, final_lid_l);
    
    // Trigger DMA transfer
    gc9a01a_update(tft);
}