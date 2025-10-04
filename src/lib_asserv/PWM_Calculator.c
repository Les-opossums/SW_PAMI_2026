#include "lib_asserv.h"

float last_valid_mesure1 = 0, last_valid_mesure2 = 0, last_valid_mesure3 = 0;

float seuil_rejet = 50.0f; // seuil de rejet des mesures

void Asserv_PWM_calculator(ESC_Command *commande) {
	float measure1 = Speed_1;
	float measure2 = Speed_2;
	float measure3 = Speed_3;

	// Vérification des mesures valides
	if (fabsf(measure1 - last_valid_mesure1) > seuil_rejet) {
		// Si la mesure est trop éloignée de la dernière mesure valide, on la rejette
		measure1 = last_valid_mesure1;
		printf("ERROR: rejected measure1\n");
	} else {
		// Sinon, on met à jour la dernière mesure valide
		last_valid_mesure1 = measure1;
	}
	if (fabsf(measure2 - last_valid_mesure2) > seuil_rejet) {
		// Si la mesure est trop éloignée de la dernière mesure valide, on la rejette
		measure2 = last_valid_mesure2;
		printf("ERROR: rejected measure1\n");
	} else {
		// Sinon, on met à jour la dernière mesure valide
		last_valid_mesure2 = measure2;
	}
	if (fabsf(measure3 - last_valid_mesure3) > seuil_rejet) {
		// Si la mesure est trop éloignée de la dernière mesure valide, on la rejette
		measure3 = last_valid_mesure3;
		printf("ERROR: rejected measure1\n");
	} else {
		// Sinon, on met à jour la dernière mesure valide
		last_valid_mesure3 = measure3;
	}

    // maj des consignes des PID
	float err1 = Speed_Order_1 - Speed_1;
	float err2 = Speed_Order_2 - Speed_2;
	float err3 = Speed_Order_3 - Speed_3;

	// calcul des commandes
	*commande = pid_speed_processing(&pid_speed, err1, err2, err3);
}
