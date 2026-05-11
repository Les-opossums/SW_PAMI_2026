#include "../PAMI_2026.h"

static int match_state = 0;
static uint32_t timer_match = 0;
static Position Goal_Pos;
static int previous_AU_state = -1;

// --- AJOUT POUR LE SERVO ---
static bool servo_enabled = false; 

// Départ zone Jaune (regard vers l'avant de la table, Y=0)
static Position init_pos_blue = {2.6f, 1.9f, -1.57f}; 
static Position init_pos_yellow = {0.3f, 1.9f, -1.57f};
static Position init_pos;

void script_match_2_loop(void){
    // ---------------------------------------------------------
    // 1. GESTION DE L'INITIALISATION ET ARRÊT D'URGENCE (AU)
    // ---------------------------------------------------------
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
        avoidance_en = 1;      // Réactiver l'esquive après un reset
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
                printf("PAMI: Go!\n");
                timer_match = Timer_ms1;
                match_state++;
            }
            break;
            
        case 1:
            // ETAPE 1 : Attente des 15 premières secondes
            if (Timer_ms1 - timer_match >= START_PLACEMENT_TIME) {
                printf("PAMI: 15s ecoulees, repli vers la zone de depart.\n");
                match_state++;
            }
            break;

        case 2:
            // ETAPE 2 : Repli en zone de départ avec Y = 1.65
            // On garde le X et le Theta déterminés à l'initialisation
            Goal_Pos.x = init_pos.x; 
            Goal_Pos.y = 1.65f; 
            Goal_Pos.t = init_pos.t;
            avoidance_en = 1; // On sécurise le repli avec l'esquive
            motion_pos(Goal_Pos); //[cite: 5]
            match_state++;
            break;

        case 3:
            // Attente de la fin du mouvement de repli
            // Validation "à la volée" sans attendre motion_done
            float dx = Goal_Pos.x - position_robot.x; //[cite: 6]
            float dy = Goal_Pos.y - position_robot.y; //[cite: 6]
            float dist = sqrtf(dx*dx + dy*dy);
            if (dist < 0.10) { //[cite: 5]
                printf("PAMI: Repli termine. Attente de la 85e seconde...\n");
                match_state++;
            }
            break;

        case 4:
            // ETAPE 3 : Attente jusqu'à la 85ème seconde du match
            if (Timer_ms1 - timer_match >= START_MATCH_DELAY) {
                printf("PAMI: 85s ecoulees, lancement du script offensif !\n");
                match_state++;
            }
            break;
            
        case 5:
            // Mouvement 1 du script nominal : On vise le premier point
            if (IHM.team_state == JAUNE) {
                Goal_Pos.x = 0.3f; Goal_Pos.y = 1.3f; Goal_Pos.t = init_pos.t;
            } else {
                Goal_Pos.x = 2.6f; Goal_Pos.y = 1.3f; Goal_Pos.t = init_pos.t;
            }       
            avoidance_en = 0; // Désactiver l'esquive pour ce mouvement précis 
            motion_pos(Goal_Pos); //[cite: 5]
            match_state++; 
            break;
            
        case 6:
            {
                // Validation "à la volée" sans attendre motion_done
                float dx = Goal_Pos.x - position_robot.x; //[cite: 6]
                float dy = Goal_Pos.y - position_robot.y; //[cite: 6]
                float dist = sqrtf(dx*dx + dy*dy);
                
                if (dist < WAYPOINT_TOLERANCE) { 
                    printf("PAMI: Passage 1 valide a la volee.\n");
                    match_state++;
                }
            }
            break;          

        case 7:
            // Mouvement 2 du script nominal
            if (IHM.team_state == JAUNE) {
                Goal_Pos.x = 0.3f; Goal_Pos.y = 0.3f; Goal_Pos.t = init_pos.t;
            } else {
                Goal_Pos.x = 2.6f; Goal_Pos.y = 0.3f; Goal_Pos.t = init_pos.t;
            }  
            avoidance_en = 1;  
            motion_pos(Goal_Pos); //[cite: 5]
            match_state++; 
            break;

        case 8:
            {
                // Validation "à la volée" du point 2
                float dx = Goal_Pos.x - position_robot.x; //[cite: 6]
                float dy = Goal_Pos.y - position_robot.y; //[cite: 6]
                float dist = sqrtf(dx*dx + dy*dy);
                
                if (dist < WAYPOINT_TOLERANCE) {
                    printf("PAMI: Passage 2 fluide valide !\n");
                    match_state++; 
                }
            }
            break;

        case 9:
            // Mouvement 3 : Cible finale du script
            if (IHM.team_state == JAUNE) {
                Goal_Pos.x = 0.7f; Goal_Pos.y = 0.1f; Goal_Pos.t = init_pos.t;
            } else {
                Goal_Pos.x = 2.3f; Goal_Pos.y = 0.1f; Goal_Pos.t = init_pos.t;
            }   
            motion_pos(Goal_Pos); //[cite: 5]
            match_state++; 
            break;

        case 10:
            // Pour le point final, on veut un arrêt complet de l'asservissement
            if (motion_done) { //[cite: 5]
                printf("PAMI: Destination finale atteinte. Attente fin du timer...\n");
                match_state = 101; 
            }
            break;

        case 100:
        case 101:
        case 102:
            // Fin de script : on bloque ici en attendant que le timer 
            // de fin de match (géré plus haut) passe la condition > ENDGAME_TIME.
            break;

        default:
            break;
    }

    // Appel continu de la machine à état du servo
    servo_process_loop(servo_enabled); 
}