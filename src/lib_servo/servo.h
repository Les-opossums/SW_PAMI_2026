#ifndef SERVO_H
#define SERVO_H

// Définitions pour le servo sur GPIO 15
#define SERVO_PIN 15
#define SERVO_MIN_PULSE 500   // µs pour 0° (à ajuster selon ton servo)
#define SERVO_MAX_PULSE 2500  // µs pour 180°
#define SERVO_FREQ 50         // 50Hz

typedef enum {
    SERVO_IDLE,
    SERVO_MOVING_TO_MAX,
    SERVO_MOVING_TO_MIN
} ServoState;

void set_servo_angle(uint pin, float angle);
void init_pami_servo();
void servo_process_loop(bool enable);
#endif // SERVO_H