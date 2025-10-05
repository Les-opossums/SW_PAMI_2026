#include "PAMI_2026.h"

StepperMotor motors[NUM_MOTORS];
// ==============================
// Init pins et moteurs
// ==============================
void init_motors() {
    motors[0].pins.step_pin = PIN_STEP_1;
    motors[0].pins.dir_pin = PIN_DIR_1;
    // motors[0].pins.en_pin = 0; // disable pin

    motors[1].pins.step_pin = PIN_STEP_2;
    motors[1].pins.dir_pin = PIN_DIR_2;
    // motors[1].pins.en_pin = 0; // disable pin

    motors[2].pins.step_pin = PIN_STEP_3;
    motors[2].pins.dir_pin = PIN_DIR_3;
    // motors[2].pins.en_pin = 0; // disable pin

    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_init(motors[i].pins.step_pin);
        gpio_set_dir(motors[i].pins.step_pin, GPIO_OUT);
        gpio_put(motors[i].pins.step_pin, 0);

        gpio_init(motors[i].pins.dir_pin);
        gpio_set_dir(motors[i].pins.dir_pin, GPIO_OUT);
        gpio_put(motors[i].pins.dir_pin, 0);

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

uint8_t Move_Stepper_Cmd(void){
    float val_1, val_2, val_3;
    if (Get_Param_Float(&val_1)) val_1 = 0;
    if (Get_Param_Float(&val_2)) val_2 = 0;
    if (Get_Param_Float(&val_3)) val_3 = 0;

    val_1 *= 1000;
    val_2 *= 1000;
    val_3 *= 1000;

    Move_Stepper(val_1, val_2, val_3);
    return 0;
}

void Move_Stepper(int32_t delay_1, int32_t delay_2, int32_t delay_3){
    if(abs(delay_1) < 10 || abs(delay_1) > 1000000){
        motors[0].enabled = false;
    }else{
        motors[0].step_period_us = abs(delay_1);
        motors[0].dir_pin = (delay_1 > 0) ? 1 : 0;
        motors[0].enabled = true;
    }
    
    if(abs(delay_2) < 10 || abs(delay_2) > 1000000){
        motors[1].enabled = false;
    }else{
        motors[1].step_period_us = abs(delay_2);
        motors[1].dir_pin = (delay_2 > 0) ? 1 : 0;
        motors[1].enabled = true;
    }

    if(abs(delay_3) < 10 || abs(delay_3) > 1000000){
        motors[2].enabled = false;
    }else{
        motors[2].step_period_us = abs(delay_3);
        motors[2].dir_pin = (delay_3 > 0) ? 1 : 0;
        motors[2].enabled = true;
    }

}