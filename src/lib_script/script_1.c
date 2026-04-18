#include "../PAMI_2026.h"

static int match_state = 0;
static uint32_t timer_match = 0;
static Position Goal_Pos;
static int previous_AU_state = -1;

// Départ zone Jaune (regard vers l'avant de la table, Y=0)
static Position init_pos_blue = {2.6f, 1.8f, -1.57}; 
static Position init_pos_yellow = {0.3f, 1.8f, -1.57};
static Position init_pos;

void script_match_1_loop(void){
    if (IHM.au_state != previous_AU_state && IHM.au_state == 0) {
        previous_AU_state = IHM.au_state;
        if(IHM.team_state == 0) {
            printf("PAMI: AU mode ON - Equipe BLEUE\n");
            init_pos = init_pos_blue;
        } else {
            printf("PAMI: AU mode ON - Equipe JAUNE\n");
            init_pos = init_pos_yellow;
        }
        Fusion_Init(init_pos.x, init_pos.y, init_pos.t); 
    } else {
        previous_AU_state = IHM.au_state;
    }

    if (IHM.au_state == 0 || !IHM.start_match) {
        match_state = 0;
        return;
    }
    
    if (match_state > 0 && match_state < 100) {
        if ((Timer_ms1 - timer_match) >= ENDGAME_TIME) {
            motion_free(); 
            match_state = 100; 
        }
    }

    switch (match_state) {
        case 0:
            if (IHM.start_match) {
                printf("PAMI: Go (Mode Petit Plateau) !\n");
                timer_match = Timer_ms1;
                match_state++;
            }
            break;
            
        case 1:
            if (Timer_ms1 - timer_match >= START_MATCH_DELAY) {
                match_state++;
            }
            break;
            
        case 2:
            // Mouvement 1 : Avancer de 50cm vers l'avant
            if (IHM.team_state == 0) {
                Goal_Pos.x = init_pos.x;        // Reste à 2.6
                Goal_Pos.y = init_pos.y - 0.5f; // Va à 1.3
                Goal_Pos.t = init_pos.t;
            } else {
                Goal_Pos.x = 0.3f;        // Reste à 0.3
                Goal_Pos.y = 1.4f; // Va à 1.3
                Goal_Pos.t = init_pos.t;
            }        
            motion_pos(Goal_Pos);
            match_state++; 
            break;
            
        case 3:
            if (motion_done) {
                printf("PAMI: Mouvement Y termine.\n");
                match_state++;
            }
            break;

        case 4:
            // Mouvement 2 : Décalage de 40cm vers la droite
            if (IHM.team_state == 0) {
                Goal_Pos.x = init_pos.x;        // Reste à 2.6
                Goal_Pos.y = init_pos.y - 0.5f; // Va à 1.3
                Goal_Pos.t = init_pos.t;
            } else {
                Goal_Pos.x = 0.7f;        // Reste à 0.3
                Goal_Pos.y = 0.1f; // Va à 1.3
                Goal_Pos.t = init_pos.t;
            }   
            motion_pos(Goal_Pos);
            match_state++; 
            break;

        case 5:
            if (motion_done) {
                printf("PAMI: Parcours de test valide !\n");
                match_state = 101; 
            }
            break;

        case 100:
            match_state = 101;
            break;

        case 101:
            break;

        default:
            break;
    }
}