#include "../PAMI_2026.h"

static int match_state = 0;
static uint32_t timer_match = 0;
static Position Goal_Pos;
static int previous_AU_state = -1;

// --- AJOUT POUR LE SERVO ---
static bool servo_enabled = false; 


// Départ zone Jaune (regard vers l'avant de la table, Y=0)
static Position init_pos_blue = {2.6f, 1.9f, -1.57}; 
static Position init_pos_yellow = {0.3f, 1.9f, -1.57};
static Position init_pos;

void script_match_2_loop(void){
    // Gestion de l'initialisation par l'Arrêt d'Urgence (AU)
    if (IHM.au_state != previous_AU_state && IHM.au_state == 0) {
        previous_AU_state = IHM.au_state;
        if(IHM.team_state == JAUNE) {
            printf("PAMI: AU mode ON - Equipe JAUNE\n");
            init_pos = init_pos_yellow;
        } else {
            printf("PAMI: AU mode ON - Equipe BLEUE\n");
            init_pos = init_pos_blue;
        }
        Fusion_Init(init_pos.x, init_pos.y, init_pos.t); //[cite: 7]
        servo_enabled = false; // Sécurité : servo off au reset
        avoidance_en = 1; // Réactiver l'esquive après un reset
    } else {
        previous_AU_state = IHM.au_state;
    }

    if (IHM.au_state == 0 || !IHM.start_match) {
        match_state = 0;
        servo_enabled = false;
        return;
    }
    
    // --- GESTION DU TIMING SERVO ET FIN DE MATCH ---
    if (match_state > 0 && match_state < 100) {
        uint32_t elapsed = Timer_ms1 - timer_match;

        // Activation du servo 10s avant la fin
        if (elapsed >= SERVO_ACTIVATION_TIME) {
            if (!servo_enabled) {
                printf("PAMI: 10s restantes - Activation du servo !\n");
                servo_enabled = true;
            }
        }

        // Arrêt total à la fin du temps réglementaire
        if (elapsed >= ENDGAME_TIME) {
            motion_free(); //[cite: 5]
            servo_enabled = false; // Optionnel : arrêter le servo à la fin précise
            match_state = 100; 
        }
    }

    switch (match_state) {
        case 0:
            if (IHM.start_match) {
                printf("PAMI: Go!\n");
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
            // Mouvement 1 : On fonce vers la première étape
            if (IHM.team_state == JAUNE) {
                Goal_Pos.x = 0.3f; Goal_Pos.y = 1.3f; Goal_Pos.t = init_pos.t;
            } else {
                Goal_Pos.x = 2.6f; Goal_Pos.y = 1.3f; Goal_Pos.t = init_pos.t;
            }       
            avoidance_en = 0; 
            motion_pos(Goal_Pos);
            match_state++; 
            break;
            
        case 3:
            {
                // Calcul de la distance restante jusqu'à Goal_Pos
                float dx = Goal_Pos.x - position_robot.x; //[cite: 6]
                float dy = Goal_Pos.y - position_robot.y; //[cite: 6]
                float dist = sqrtf(dx*dx + dy*dy);
                
                // On n'attend pas motion_done, on anticipe !
                if (dist < WAYPOINT_TOLERANCE) { 
                    printf("PAMI: Passage Y valide a la volee.\n");
                    match_state++;
                }
            }
            break;          

        case 4:
            // Mouvement 2 : On change de cap sans s'arrêter
            if (IHM.team_state == JAUNE) {
                Goal_Pos.x = 0.3f; Goal_Pos.y = 0.3f; Goal_Pos.t = init_pos.t;
            } else {
                Goal_Pos.x = 2.6f; Goal_Pos.y = 0.3f; Goal_Pos.t = init_pos.t;
            }  
            avoidance_en = 1;  
            motion_pos(Goal_Pos);
            match_state++; 
            break;

        case 5:
            {
                // Vérification du deuxième point de passage
                float dx = Goal_Pos.x - position_robot.x; //[cite: 6]
                float dy = Goal_Pos.y - position_robot.y; //[cite: 6]
                float dist = sqrtf(dx*dx + dy*dy);
                
                if (dist < WAYPOINT_TOLERANCE) {
                    printf("PAMI: Mouvement 2 fluide valide !\n");
                    match_state++; 
                }
            }
            break;

        case 6:
            // Mouvement 3 : C'est le point d'arrivée final
            if (IHM.team_state == JAUNE) {
                Goal_Pos.x = 0.7f; Goal_Pos.y = 0.1f; Goal_Pos.t = init_pos.t;
            } else {
                Goal_Pos.x = 2.3f; Goal_Pos.y = 0.1f; Goal_Pos.t = init_pos.t;
            }   
            motion_pos(Goal_Pos);
            match_state++; 
            break;

        case 7:
            // Pour le DERNIER point, on veut s'arrêter précisément, 
            // donc ici il est judicieux de garder motion_done !
            if (motion_done) { //
                printf("PAMI: Destination finale atteinte.\n");
                match_state = 101; 
            }
            break;

        case 100:
            match_state = 101;
            break;

        default:
            break;
    }

    servo_process_loop(servo_enabled); 
}