#include "../PAMI_2026.h"

int match_state = 0;
int timer_match = 0;
int timer_match_delay = 85000; // 85 secondes (Coupe de France)
int timer_match_delay_endgame = 100000; 
Position Goal_Pos;

void script_loop(void) {
    switch (IHM.ID) {
        case 1:
            script_match_1_loop();
            break;
        case 2:
            script_match_2_loop();
            break;
        case 3:
            script_match_3_loop();
            break;
        case 4:
            script_match_4_loop();
            break;
        case 5:
            script_match_5_loop();
            break;
        case 6:
            script_match_6_loop();
            break;
        default:
            // Si l'ID est 0 ou non configuré, le robot reste immobile par sécurité
            // On peut aussi faire clignoter les LEDs en rouge pour signaler l'erreur
            break;
    }
}