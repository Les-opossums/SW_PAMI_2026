#include "../PAMI_2026.h"

static int match_state = 0;
static uint32_t timer_match = 0;

static Position Goal_Pos;

static int previous_AU_state = -1;

static Position init_pos = {0.2f, 0.2f, 1.5f};

void script_match_1_loop(void){
    if (IHM.au_state == 0 || !IHM.start_match) {
        match_state = 0;
        return;
    }

    if (IHM.au_state != previous_AU_state && IHM.au_state == 0) {
        previous_AU_state = IHM.au_state;
        Fusion_Init(init_pos.x, init_pos.y, init_pos.t); // reset position estimation
    } else {
        previous_AU_state = IHM.au_state;
    }

    switch (match_state) {
        case 0:
            if (IHM.start_match) {
                printf("PAMI 1: Go !\n");
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
            Goal_Pos.x = 0.5;
            Goal_Pos.y = 1.0;
            Goal_Pos.t = 1.5708; 
            motion_pos(Goal_Pos);
            match_state++; 
            break;
        case 3:
            if (motion_done) {
                printf("PAMI 1: Goal reached !\n");
                match_state++;
            }
            break;
        case 4:
            Goal_Pos.x = 0.5;
            Goal_Pos.y = 0.2;
            Goal_Pos.t = 1.5708; 
            motion_pos(Goal_Pos);
            match_state++; 
            break;
        case 5:
            if (motion_done) {
                printf("PAMI 1: Goal reached !\n");
                match_state++;
            }
            break;
        default:
            break;
    }
}