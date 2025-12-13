/**
 * @file minion_eye.h
 * @brief Logic for the Minion Eye Animation.
 * * This module handles the physics, blinking, and rendering of the eye.
 * It uses the generic GC9A01A driver to display the results.
 */

#ifndef MINION_EYE_H
#define MINION_EYE_H

// --- Eye Specific Colors ---
#define MINION_YELLOW    0xFFE0 
#define EYE_WHITE        0xFFFF
#define PUPIL_BLACK      0x0000
#define IRIS_DARK_BROWN  0x5140 
#define IRIS_MAIN_BROWN  0x8200 
#define IRIS_LIGHT_BROWN 0xA302 

/**
 * @brief Available emotions for the eye.
 */
typedef enum {
    EMOTION_NORMAL,
    EMOTION_HAPPY,
    EMOTION_TIRED,
    EMOTION_SUSPICIOUS,
    EMOTION_AUTO // Internally picks random emotions
} MinionEmotion;

/**
 * @brief Initialize the eye logic.
 * @param driver Pointer to the initialized GC9A01A driver handle.
 */
void minion_eye_init(gc9a01a_t *driver);

/**
 * @brief The Main Non-Blocking Update Task.
 * * Call this function in your main `while(1)` loop.
 * It is efficient and non-blocking:
 * - If DMA is busy: Returns immediately (0us cost).
 * - If 33ms haven't passed: Returns immediately.
 * - If ready: Renders 1 frame (~20ms), starts DMA, and returns.
 */
void minion_eye_update_non_blocking(void);

/**
 * @brief Force a specific emotion.
 * @param emotion The emotion to set.
 */
void minion_eye_set_emotion(MinionEmotion emotion);

#endif // MINION_EYE_H