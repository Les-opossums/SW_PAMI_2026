#include "lib_asserv.h"


void Asserv_PWM_calculator(MOTOR_Command *commande) {
    // maj des consignes des PID
	float err1 = Speed_Order_1 - speed_1;
	float err2 = Speed_Order_2 - speed_2;
	float err3 = Speed_Order_3 - speed_3;

	// calcul des commandes
	*commande = pid_speed_processing(&pid_speed, err1, err2, err3);
}
