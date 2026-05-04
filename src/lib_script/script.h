#ifndef LIB_SCRIPT_H
#define LIB_SCRIPT_H

#define START_MATCH_DELAY 3000 // 10 secondes après le démarrage du robot
#define ENDGAME_TIME 15000 // 30 secondes après le démarrage du robot
#define SERVO_ACTIVATION_TIME (ENDGAME_TIME - 10000) // 90 secondes en ms (soit 10s avant la fin de 100s)

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