#include "PAMI_2026.h"

StepperMotor motors[NUM_MOTORS];
// ==============================
// Init pins et moteurs
// ==============================
void init_motors() {
    motors[0].pins.step_pin = PIN_STEP_1;
    motors[0].pins.dir_pin = PIN_DIR_1;
    motors[0].pins.en_pin = 0; // disable pin

    // motors[1].pins.step_pin = PIN_STEP_2;
    // motors[1].pins.dir_pin = PIN_DIR_2;
    // motors[1].pins.en_pin = 0; // disable pin

    // motors[2].pins.step_pin = PIN_STEP_3;
    // motors[2].pins.dir_pin = PIN_DIR_3;
    // motors[2].pins.en_pin = 0; // disable pin

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].current_position = 0;
        motors[i].enabled = false;
        motors[i].last_step_time_us = 0;
        motors[i].step_period_us = 1000; // Default to 1000us (1ms) between steps
    }
}

void Move_Loop(){
    uint64_t current_time_us = Timer_us1;
    for (int i=0; i<NUM_MOTORS; i++){
        if(motors[i].enabled){
            if(current_time_us - motors[i].last_step_time_us >= motors[i].step_period_us){
                motors[i].last_step_time_us = current_time_us; // Update last step time
                motors[i].step_pin = !motors[i].step_pin; // Toggle step pin
                gpio_put(motors[i].pins.step_pin, motors[i].step_pin);
                gpio_put(motors[i].pins.dir_pin, motors[i].dir_pin);
                if(motors[i].step_pin){
                    if(motors[i].dir_pin){
                        motors[i].current_position++; // Move forward
                    } else {
                        motors[i].current_position--; // Move backward
                    }
                }
            }
        } else {
            motors[i].last_step_time_us = current_time_us;
        }
    }
}
