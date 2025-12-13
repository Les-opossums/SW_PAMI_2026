#ifndef MINION_EYE_H
#define MINION_EYE_H

typedef enum {
    EMOTION_NORMAL,
    EMOTION_HAPPY,
    EMOTION_TIRED,
    EMOTION_SUSPICIOUS,
    EMOTION_AUTO 
} MinionEmotion;

/**
 * @brief Initialize the minion eye state.
 */
void minion_eye_init(gc9a01a_t *driver);

/**
 * @brief THE MAIN NON-BLOCKING TASK.
 * Call this inside your main while(1) loop.
 * * It will:
 * 1. Check if DMA is busy. (If yes, it returns IMMEDIATELY).
 * 2. If free, it checks if enough time passed for next frame (e.g. 30FPS).
 * 3. If time, it renders ONE frame and starts the DMA background transfer.
 */
void minion_eye_update_non_blocking(void);

/**
 * @brief Force a specific emotion.
 */
void minion_eye_set_emotion(MinionEmotion emotion);

#endif