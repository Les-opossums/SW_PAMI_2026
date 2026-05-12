#include "../PAMI_2026.h"

static int match_state = 0;
static uint32_t timer_match = 0;
static Position Goal_Pos;
static int previous_AU_state = -1;

static bool servo_enabled = false; 

// Départ zone Jaune (regard vers l'avant de la table, Y=0)
static Position init_pos_blue = {2.65f, 1.9f, -1.57f}; 
static Position init_pos_yellow = {0.35f, 1.9f, -1.57f};
static Position init_pos;

void script_match_2_loop(void){
    if (IHM.au_state != previous_AU_state && IHM.au_state == 0) {
        previous_AU_state = IHM.au_state;
        if(IHM.team_state == BLEU) {
            printf("PAMI: AU mode ON - Equipe BLEUE\n");
            init_pos = init_pos_blue;
        } else {
            printf("PAMI: AU mode ON - Equipe JAUNE\n");
            init_pos = init_pos_yellow;
        }
        Fusion_Init(init_pos.x, init_pos.y, init_pos.t); 
        servo_enabled = false;
        avoidance_en = 1;
    } else {
        previous_AU_state = IHM.au_state;
    }

    if (IHM.au_state == 0 || !IHM.start_match) {
        match_state = 0;
        servo_enabled = false;
        return;
    }
    
    // ---------------------------------------------------------
    // 2. GESTION DU TIMING GLOBAL DU MATCH (Indépendant des déplacements)
    // ---------------------------------------------------------
    if (match_state > 0) {
        uint32_t elapsed = Timer_ms1 - timer_match;

        // Activation du servo X secondes avant la fin
        if (elapsed >= SERVO_ACTIVATION_TIME) {
            if (!servo_enabled) {
                printf("PAMI: 10s restantes - Activation du servo !\n");
                servo_enabled = true;
            }
        }

        // Arrêt total à la fin du temps réglementaire (ex: 100s)
        if (elapsed >= ENDGAME_TIME) {
            if (match_state != 102) { 
                motion_free(); //[cite: 5]
                printf("PAMI: Fin du match, coupure des moteurs !\n");
                
                // CRITIQUE : On ne coupe pas le servo_enabled ici !
                // Il restera à "true" indéfiniment pour maintenir l'action finale.
                
                match_state = 102; // Verrouille la machine à états de déplacement
            }
        }
    }

    // ---------------------------------------------------------
    // 3. MACHINE À ÉTATS DE DÉPLACEMENT
    // ---------------------------------------------------------

    switch (match_state) {
        case 0:
            if (IHM.start_match) {
                printf("PAMI: Go !\n");
                timer_match = Timer_ms1;
                servo_enabled = false;
                avoidance_en = 0; // On force la désactivation de l'évitement dans la zone de départ
                match_state++;
            }
            break;

        case 1:
            if (Timer_ms1 - timer_match >= START_PLACEMENT_TIME) {
                match_state++;
            }
            break;

        case 2:
            // on se replace dans la zone pour préparer le départ
            if (IHM.team_state == BLEU) {
                Goal_Pos.x = 2.65f;        // Reste à 2.65
            } else {
                Goal_Pos.x = 0.35f;        // Reste à 0.35
            }        
            Goal_Pos.y = 1.65f;            // Va à 1.65
            Goal_Pos.t = init_pos.t;
            motion_pos(Goal_Pos);
            match_state++; 
            break;
            
        case 3:
            if (Timer_ms1 - timer_match >= START_MATCH_DELAY) {
                match_state++;
            }
            break;
            
        case 4:
            if (IHM.team_state == BLEU) {
                Goal_Pos.x = 2.2f;        // Va à 2.2
            } else {
                Goal_Pos.x = 0.8f;        // Va à 0.8
            }   
            Goal_Pos.y = 0.8f;            // Reste à 0.8
            Goal_Pos.t = init_pos.t;
            motion_pos(Goal_Pos);
            avoidance_en = 1;
            match_state++; 
            break;

        case 5:
            // Calcul de la distance restante[cite: 5, 6]
            float dx = Goal_Pos.x - position_robot.x;
            float dy = Goal_Pos.y - position_robot.y;
            float dist = sqrtf(dx * dx + dy * dy);

            // Désactivation de l'évitement à moins de 10cm
            if (dist <= 0.10f) {
                avoidance_en = 0;
            }

            // Fin du parcours : ici on attend l'arrêt complet
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

    // Appel continu de la machine à état du servo
    servo_process_loop(servo_enabled); 
}