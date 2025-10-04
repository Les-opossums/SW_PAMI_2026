#include "lib_asserv.h"


void Asserv_PWM_calculator(MOTOR_Command *commande) {
	if (Speed_Order_1 < 0.001) {
		commande->command1 = 0;
	} else {
    	commande->command1 = (int32_t)(1000000.0f * DEFAULT_SIZE_WHEEL * M_PI / (Speed_Order_1 * DEFAULT_STEPS_PER_REV));
	}
	if (Speed_Order_2 < 0.001) {
		commande->command2 = 0;
	} else {
		commande->command2 = (int32_t)(1000000.0f * DEFAULT_SIZE_WHEEL * M_PI / (Speed_Order_2 * DEFAULT_STEPS_PER_REV));
	}
	if (Speed_Order_3 < 0.001) {
		commande->command3 = 0;
	} else {
		commande->command3 = (int32_t)(1000000.0f * DEFAULT_SIZE_WHEEL * M_PI / (Speed_Order_3 * DEFAULT_STEPS_PER_REV));
	}
}