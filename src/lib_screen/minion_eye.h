#ifndef MINION_EYE_H
#define MINION_EYE_H

/**
 * @brief Initialize the minion eye logic and link it to the display driver.
 * @param driver Pointer to the initialized GC9A01A driver.
 */
void minion_eye_init(gc9a01a_t *driver);

/**
 * @brief Run one iteration of the eye logic (movement, emotion, blinking, rendering).
 * call this inside your main while(1) loop.
 */
void minion_eye_loop(void);

#endif // MINION_EYE_H