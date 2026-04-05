#ifndef LIB_SCRIPT_H
#define LIB_SCRIPT_H

#define START_MATCH_DELAY 10000 // 10 secondes après le démarrage du robot
#define ENDGAME_TIME 30000 // 30 secondes après le démarrage du robot

// Function declarations
void script_loop(void);

void script_match_1_loop(void);
void script_match_2_loop(void);
void script_match_3_loop(void);
void script_match_4_loop(void);
void script_match_5_loop(void);
void script_match_6_loop(void);

#endif // LIB_SCRIPT_H