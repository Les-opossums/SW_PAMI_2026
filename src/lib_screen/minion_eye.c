/**
 * @file minion_eye.c
 * @brief Physics and Rendering logic for the Minion Eye.
 */

 #include "../PAMI_2026.h"
// --- Config ---
#define CENTER_X 120
#define CENTER_Y 120
#define MAX_PUPIL_DIST 35 
#define MAX_LID_HEIGHT 115.0f
#define FRAME_INTERVAL_MS 33 // Target ~30 FPS

// --- State Variables ---
static gc9a01a_t *tft_handle; 
static MinionEmotion forced_emotion = EMOTION_AUTO;

// Physics
static float cur_x = CENTER_X, cur_y = CENTER_Y;
static float start_x = CENTER_X, start_y = CENTER_Y;
static float target_x = CENTER_X, target_y = CENTER_Y;
static bool is_moving = false;
static uint32_t next_move_time = 0;
static uint32_t move_start_time = 0, move_duration = 0;

// Lids & Blink
static float cur_lid_u = 20.0f, cur_lid_l = 15.0f;
static bool is_blinking = false;
static uint32_t next_blink_time = 0, blink_start_time = 0;

// Auto Emotion Logic
static MinionEmotion current_internal_emotion = EMOTION_NORMAL;
static uint32_t next_emotion_time = 0;
static uint32_t next_render_time = 0;

// Target lid heights for emotions: {Upper, Lower}
static const float emotion_targets[4][2] = {
    {20.0f, 15.0f}, // NORMAL
    {0.0f,  0.0f},  // HAPPY
    {65.0f, 40.0f}, // TIRED
    {85.0f, 85.0f}  // SUSPICIOUS
};

// --- Helper Math ---
static float smooth_move(float current, float target, float speed) {
    if (fabs(current - target) < speed) return target;
    return current + (target - current) * speed;
}

static float ease_out_cubic(float t) { 
    return 1.0f - powf(1.0f - t, 3.0f); 
}

// --- Specific Rendering Logic ---
// We render directly to the driver's buffer using its primitives or raw access
static void render_frame(uint16_t *buffer, int16_t pupil_x, int16_t pupil_y, float lid_u, float lid_l) {
    // 1. Clear Screen
    gc9a01a_fill_screen(GC9A01A_BLACK);

    // 2. Draw Eye Ball & Iris using Driver Primitives
    gc9a01a_fill_circle(buffer, 120, 120, 120, EYE_WHITE);
    gc9a01a_fill_circle(buffer, pupil_x, pupil_y, 62, IRIS_DARK_BROWN);
    gc9a01a_fill_circle(buffer, pupil_x, pupil_y, 56, IRIS_MAIN_BROWN);
    gc9a01a_fill_circle(buffer, pupil_x, pupil_y, 38, IRIS_LIGHT_BROWN);
    gc9a01a_fill_circle(buffer, pupil_x, pupil_y, 26, PUPIL_BLACK);
    gc9a01a_fill_circle(buffer, pupil_x + 14, pupil_y - 14, 9, EYE_WHITE);

    // 3. Draw Eyelids (Complex Parabolic Shapes)
    // Direct buffer manipulation for speed/complexity
    uint16_t swapped_yellow = __builtin_bswap16(MINION_YELLOW);
    float curve_intensity = 25.0f; 
    float normalize_factor = 1.0f / (120.0f * 120.0f); 

    if (lid_u < 5 && lid_l < 5) return; // Optimization: Lids open

    for (int y = 0; y < 240; y++) {
        for (int x = 0; x < 240; x++) {
            float dx = (float)(x - 120);
            float curve_offset = (dx * dx) * normalize_factor * curve_intensity;

            // Upper Lid
            if (y < (lid_u + curve_offset)) {
                buffer[y * 240 + x] = swapped_yellow;
            }
            // Lower Lid
            else if (y > (240.0f - lid_l - curve_offset)) {
                buffer[y * 240 + x] = swapped_yellow;
            }
        }
    }
}

// --- Public API Implementation ---

void minion_eye_init(gc9a01a_t *driver) {
    tft_handle = driver;
    srand(to_us_since_boot(get_absolute_time()));
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    next_move_time = now + 1000;
    next_emotion_time = now + 3000;
    next_blink_time = now + 2000;
    next_render_time = now;
}

void minion_eye_update_non_blocking(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // 1. Hardware Check
    if (gc9a01a_is_busy()) return;

    // 2. Framerate Check
    if (now < next_render_time) return;

    // --- Physics Update ---
    MinionEmotion target_emo = forced_emotion;
    if (target_emo == EMOTION_AUTO) {
        if (now >= next_emotion_time) {
            int pick = rand() % 10;
            if (pick < 4) current_internal_emotion = EMOTION_NORMAL;
            else if (pick < 6) current_internal_emotion = EMOTION_HAPPY;
            else if (pick < 8) current_internal_emotion = EMOTION_TIRED;
            else current_internal_emotion = EMOTION_SUSPICIOUS;
            next_emotion_time = now + 3000 + (rand() % 5000);
        }
        target_emo = current_internal_emotion;
    }

    // Saccades (Eye Movement)
    if (!is_moving) {
        if (now >= next_move_time) {
            is_moving = true; 
            move_start_time = now;
            move_duration = 80 + (rand() % 120);
            
            float angle = (rand() % 360) * 0.01745f;
            float range_mod = (target_emo == EMOTION_SUSPICIOUS) ? 0.6f : 1.0f;
            float dist = (rand() % MAX_PUPIL_DIST) * range_mod;
            
            start_x = cur_x; start_y = cur_y;
            target_x = CENTER_X + cosf(angle) * dist;
            target_y = CENTER_Y + sinf(angle) * dist;
            next_move_time = now + move_duration + 300 + (rand() % 2000);
        }
    } else {
        float t = (float)(now - move_start_time);
        if (t >= move_duration) {
            cur_x = target_x; cur_y = target_y; is_moving = false;
        } else {
            float val = ease_out_cubic(t / move_duration);
            cur_x = start_x + (target_x - start_x) * val;
            cur_y = start_y + (target_y - start_y) * val;
        }
    }

    // Lids
    cur_lid_u = smooth_move(cur_lid_u, emotion_targets[target_emo][0], 0.15f);
    cur_lid_l = smooth_move(cur_lid_l, emotion_targets[target_emo][1], 0.15f);

    float final_u = cur_lid_u; 
    float final_l = cur_lid_l;

    // Blinking
    if (!is_blinking && now >= next_blink_time) { 
        is_blinking = true; blink_start_time = now; 
    }
    if (is_blinking) {
        uint32_t t = now - blink_start_time;
        uint32_t dur = 180;
        if (t >= dur) { 
            is_blinking = false; 
            next_blink_time = now + 800 + (rand() % 4000); 
        } else {
            float add = (t < dur/2) ? ((float)t/(dur/2.0f)) : ((float)(dur-t)/(dur/2.0f));
            add *= MAX_LID_HEIGHT;
            final_u = fminf(MAX_LID_HEIGHT, cur_lid_u + add);
            final_l = fminf(MAX_LID_HEIGHT, cur_lid_l + add);
        }
    }

    // --- Render to Buffer ---
    // Note: We access gc9a01a_draw_buffer via the driver header external declaration
    render_frame(gc9a01a_draw_buffer, (int16_t)cur_x, (int16_t)cur_y, final_u, final_l);

    // --- Send via DMA ---
    gc9a01a_update_async(tft_handle);

    // --- Schedule Next ---
    next_render_time = now + FRAME_INTERVAL_MS;
}

void minion_eye_set_emotion(MinionEmotion emotion) {
    forced_emotion = emotion;
}