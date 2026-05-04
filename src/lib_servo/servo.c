#include "../PAMI_2026.h"


// Variables de contrôle
ServoState current_state = SERVO_IDLE;
uint32_t last_move_time = 0;

// Paramètres de mouvement
uint16_t move_delay_ms = 1000; // Temps entre deux positions

/**
 * Convertit un angle (0-180) en duty cycle PWM pour le Pico
 */
void set_servo_angle(uint pin, float angle) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    // Calcul du pulse en µs
    uint32_t pulse_width = SERVO_MIN_PULSE + (uint32_t)((angle / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
    // Sur un Pico à 125MHz avec un wrap à 20000 (pour 50Hz avec diviseur 125)
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin), pulse_width);
}

void init_pami_servo() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    
    pwm_config config = pwm_get_default_config();
    // Horloge à 125MHz / 125 = 1MHz (1 tic = 1µs)
    pwm_config_set_clkdiv(&config, 125.0f);
    // Wrap à 20000 tics = 20ms (50Hz)
    pwm_config_set_wrap(&config, 20000);
    
    pwm_init(slice_num, &config, true);
}

/**
 * Machine à état à appeler dans ta boucle principale
 */
void servo_process_loop(bool enable) {
    if (!enable) {
        current_state = SERVO_IDLE;
        return;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());

    switch (current_state) {
        case SERVO_IDLE:
            set_servo_angle(SERVO_PIN, 0);
            last_move_time = now;
            current_state = SERVO_MOVING_TO_MAX;
            break;

        case SERVO_MOVING_TO_MAX:
            if (now - last_move_time > move_delay_ms) {
                set_servo_angle(SERVO_PIN, 180);
                last_move_time = now;
                current_state = SERVO_MOVING_TO_MIN;
            }
            break;

        case SERVO_MOVING_TO_MIN:
            if (now - last_move_time > move_delay_ms) {
                set_servo_angle(SERVO_PIN, 0);
                last_move_time = now;
                current_state = SERVO_MOVING_TO_MAX;
            }
            break;
    }
}