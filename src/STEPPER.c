#include "PAMI_2026.h"

// ==============================
// Init pins et moteurs
// ==============================
void init_motors() {
    // for(int i = 0; i < NUM_MOTORS; i++) {
    //     motors[i].pins = motor_pins[i];
    //     motors[i].speed = 0;
    //     motors[i].enabled = false;
    //     motors[i].last_step_time_us = 0;
    //     motors[i].step_period_us = 1000000; // par défaut très lent

    //     gpio_init(motors[i].pins.step_pin);
    //     gpio_set_dir(motors[i].pins.step_pin, true);

    //     gpio_init(motors[i].pins.dir_pin);
    //     gpio_set_dir(motors[i].pins.dir_pin, true);

    //     gpio_init(motors[i].pins.en_pin);
    //     gpio_set_dir(motors[i].pins.en_pin, true);

    //     // Désactiver le moteur au départ (LOW = enable pour A4988)
    //     gpio_put(motors[i].pins.en_pin, true);
    //     gpio_put(motors[i].pins.step_pin, false);
    // }
}

