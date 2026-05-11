#include "../PAMI_2026.h"

// Variables de contrôle
ServoState current_state = SERVO_IDLE;
uint32_t last_move_time = 0;

// On utilise les constantes définies dans ton PAMI_2026.h pour les pulses
// SERVO_MIN_PULSE (souvent 500) et SERVO_MAX_PULSE (souvent 2500)
void set_servo_angle(uint pin, float angle) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    // Protection pour rester entre 0 et 180°
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint32_t pulse_width = SERVO_MIN_PULSE + (uint32_t)((angle / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), pulse_width);
}

void init_pami_servo() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    
    pwm_config config = pwm_get_default_config();
    // Diviseur à 125.0f pour avoir 1MHz (si le Pico est à 125MHz) -> 1 tick = 1µs
    pwm_config_set_clkdiv(&config, 125.0f);
    // Wrap à 20000 pour avoir une période de 20ms (50Hz), standard servo
    pwm_config_set_wrap(&config, 20000);
    
    pwm_init(slice_num, &config, true);
    
    // Position initiale de sécurité
    set_servo_angle(SERVO_PIN, 0);
    current_state = SERVO_IDLE;
}

void servo_process_loop(bool enable) {
    if (!enable) {
        if (current_state != SERVO_IDLE) {
            set_servo_angle(SERVO_PIN, 0);
            current_state = SERVO_IDLE;
        }
        return;
    }

    uint32_t now = Timer_ms1; // Utilise le timer global de ton projet

    switch (current_state) {
        case SERVO_IDLE:
            set_servo_angle(SERVO_PIN, 0);
            last_move_time = now;
            current_state = SERVO_MOVING_TO_MAX;
            break;

        case SERVO_MOVING_TO_MAX:
            if (now - last_move_time >= 1000) {
                set_servo_angle(SERVO_PIN, 180);
                last_move_time = now; // CRUCIAL : On reset le timer ici
                current_state = SERVO_MOVING_TO_MIN;
            }
            break;

        case SERVO_MOVING_TO_MIN:
            if (now - last_move_time >= 1000) {
                set_servo_angle(SERVO_PIN, 0);
                last_move_time = now; // CRUCIAL : On reset le timer ici
                current_state = SERVO_MOVING_TO_MAX;
            }
            break;
    }
}