#include "../PAMI_2026.h"

#define CENTER_X 120
#define CENTER_Y 120
#define MAX_PUPIL_DIST 35 
#define MAX_LID_HEIGHT 115.0f
#define FRAME_INTERVAL_MS 33 // ~30 FPS limit

static gc9a01a_t *tft; 
static MinionEmotion forced_emotion = EMOTION_AUTO;

// Physics State
static float cur_x = CENTER_X, cur_y = CENTER_Y;
static float start_x = CENTER_X, start_y = CENTER_Y;
static float target_x = CENTER_X, target_y = CENTER_Y;
static bool is_moving = false;
static uint32_t next_move_time = 0;
static uint32_t move_start_time = 0, move_duration = 0;

static float cur_lid_u = 20.0f, cur_lid_l = 15.0f;
static bool is_blinking = false;
static uint32_t next_blink_time = 0, blink_start_time = 0;

static MinionEmotion current_internal_emotion = EMOTION_NORMAL;
static uint32_t next_emotion_time = 0;
static uint32_t next_render_time = 0;

static const float emotion_targets[4][2] = {
    {20.0f, 15.0f}, {0.0f, 0.0f}, {65.0f, 40.0f}, {85.0f, 85.0f}
};

static float smooth_move(float current, float target, float speed) {
    if (fabs(current - target) < speed) return target;
    return current + (target - current) * speed;
}
static float ease_out_cubic(float t) { return 1.0f - powf(1.0f - t, 3.0f); }

// --- API ---

void minion_eye_init(gc9a01a_t *driver) {
    tft = driver;
    srand(to_us_since_boot(get_absolute_time()));
    uint32_t now = to_ms_since_boot(get_absolute_time());
    next_move_time = now + 1000;
    next_emotion_time = now + 3000;
    next_blink_time = now + 2000;
    next_render_time = now;
}

void minion_eye_update_non_blocking(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // 1. EXIT CHECK: Is the hardware busy?
    if (gc9a01a_is_busy()) return;

    // 2. EXIT CHECK: Is it too early for the next frame?
    if (now < next_render_time) return;

    // --- If we are here, we are committed to rendering one frame ---
    // Update Logic (Physics)
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

    if (!is_moving) {
        if (now >= next_move_time) {
            is_moving = true; move_start_time = now;
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

    cur_lid_u = smooth_move(cur_lid_u, emotion_targets[target_emo][0], 0.15f); // Faster speed for 30fps
    cur_lid_l = smooth_move(cur_lid_l, emotion_targets[target_emo][1], 0.15f);

    float final_u = cur_lid_u, final_l = cur_lid_l;
    if (!is_blinking && now >= next_blink_time) { is_blinking = true; blink_start_time = now; }
    if (is_blinking) {
        uint32_t t = now - blink_start_time;
        uint32_t dur = 180;
        if (t >= dur) { is_blinking = false; next_blink_time = now + 800 + (rand() % 4000); }
        else {
            float add = (t < dur/2) ? ((float)t/(dur/2.0f)) : ((float)(dur-t)/(dur/2.0f));
            add *= MAX_LID_HEIGHT;
            final_u = fminf(MAX_LID_HEIGHT, cur_lid_u + add);
            final_l = fminf(MAX_LID_HEIGHT, cur_lid_l + add);
        }
    }

    // 3. Render to the BACK buffer
    gc9a01a_render_minion_frame(current_draw_buffer, (int16_t)cur_x, (int16_t)cur_y, final_u, final_l);

    // 4. Send it (returns immediately)
    gc9a01a_update_async(tft);

    // 5. Schedule next check
    next_render_time = now + FRAME_INTERVAL_MS;
}

void minion_eye_set_emotion(MinionEmotion emotion) {
    forced_emotion = emotion;
}