#include "../PAMI_2026.h"

static int match_state = 0;
static uint32_t timer_match = 0;

static Position Goal_Pos;

static int previous_AU_state = -1;

static Position init_pos = {0.2f, 0.2f, 1.5f};

void script_match_1_loop(void){
    // 1. Arrêt d'urgence ou tirette non tirée
    if (IHM.au_state != previous_AU_state && IHM.au_state == 0) {
        previous_AU_state = IHM.au_state;
        Fusion_Init(init_pos.x, init_pos.y, init_pos.t); // reset position estimation
    } else {
        previous_AU_state = IHM.au_state;
    }

    if (IHM.au_state == 0 || !IHM.start_match) {
        match_state = 0;
        return;
    }
    
    // ========================================================
    // 2. TIMEOUT GLOBAL (Fin de match)
    // On vérifie si le match a démarré (state > 0) ET qu'on n'est pas encore à la fin
    // ========================================================
    if (match_state > 0 && match_state < 100) {
        if ((Timer_ms1 - timer_match) >= ENDGAME_TIME) {
            printf("PAMI 1: Temps ecoule (30s) ! Arrêt des moteurs.\n");
            
            // On libère les moteurs (arrête le mouvement en cours)
            motion_free(); 
            
            // On force la machine d'état à aller dans la phase de fin
            match_state = 100; 
        }
    }

    // 3. Machine d'état du match
    switch (match_state) {
        case 0:
            if (IHM.start_match) {
                printf("PAMI 1: Go !\n");
                timer_match = Timer_ms1; // On lance le chronomètre !
                match_state++;
            }
            break;
            
        case 1:
            if (Timer_ms1 - timer_match >= START_MATCH_DELAY) {
                match_state++;
            }
            break;
            
        case 2:
            Goal_Pos.x = 0.5;
            Goal_Pos.y = 0.2;
            Goal_Pos.t = 1.5708; 
            motion_pos(Goal_Pos);
            match_state++; 
            break;
            
        case 3:
            if (motion_done) {
                printf("PAMI 1: Goal 1 reached !\n");
                match_state++;
            }
            break;
            
        case 4:
            Goal_Pos.x = 0.5;
            Goal_Pos.y = 1.5;
            Goal_Pos.t = 1.5708; 
            motion_pos(Goal_Pos);
            match_state++; 
            break;
            
        case 5:
            if (motion_done) {
                printf("PAMI 1: Goal 2 reached !\n");
                match_state++; // Le robot va s'arrêter ici et attendre que le temps s'écoule
            }
            break;

        // ========================================================
        // --- ACTIONS DE FIN DE MATCH (Endgame) ---
        // ========================================================
        case 100:
            printf("PAMI 1: Deploiement de l'actionneur final !\n");
            
            // Commande du servomoteur
            
            match_state = 101; // On passe à l'état suivant pour ne pas spammer la commande
            break;

        case 101:
            // Fin absolue. Le robot est inerte et attend la fin des 90s globales de la table.
            break;

        default:
            break;
    }
}