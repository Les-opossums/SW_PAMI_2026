#ifndef STEPPER_H
#define STEPPER_H

#define NUM_MOTORS 1

typedef struct {
    int step_pin;
    int dir_pin;
    int en_pin;
} MotorPins;

typedef struct {
    MotorPins pins;
    int32_t current_position; // Current position in steps
    bool enabled;
    bool step_pin; // Current state of the step pin
    bool dir_pin;  // Current state of the direction pin (0 or 1)
    uint64_t last_step_time_us; // Time of the last step in microseconds
    uint32_t step_period_us;  // Interval between steps in microseconds
} StepperMotor;

void init_motors();

#endif // STEPPER_H