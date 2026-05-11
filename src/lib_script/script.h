#ifndef LIB_SCRIPT_H
#define LIB_SCRIPT_H

#define START_MATCH_DELAY 85000
#define ENDGAME_TIME 100000
#define SERVO_ACTIVATION_TIME (ENDGAME_TIME - 10000) // 90 secondes en ms (soit 10s avant la fin de 100s)
#define START_PLACEMENT_TIME 15000 // 15 secondes pour le placement initial avant de commencer les mouvements
#define WAYPOINT_TOLERANCE 0.25f // Rayon de 25 cm pour valider le passage "à la volée"


#define JAUNE 1
#define BLEU 0

// Function declarations
void script_loop(void);

void script_match_1_loop(void);
void script_match_2_loop(void);
void script_match_3_loop(void);
void script_match_4_loop(void);
void script_match_5_loop(void);
void script_match_6_loop(void);

#endif // LIB_SCRIPT_H