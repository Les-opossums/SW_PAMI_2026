#include "PAMI_2026.h"



//global state
LD19Instance LD19;

void core1_entry() {

    LIDAR_UART_init();
    LD19_init(&LD19);
    LD19_enableFiltering(&LD19);
    LD19_setDistanceRange(&LD19, 100, 4000); // 0.1m to 4m
    while(1){
        LD19_readScan(&LD19, UART_ID);
    }
}




int main()
{
    stdio_init_all();
    sleep_ms(2000); // wait for stdio to be ready
    int sequencer = 0;

    Fusion_Init(500.0f, 500.0f, 0.0f); // init x y theta
    // Init_All();
    multicore_launch_core1(core1_entry);

    printf("PAMI-2026 ready.\n");


    while (true) {
        bool has_data = false;

        Timer_Update(); // Met à jour les timers
        int c;

        // Met à jour les moteurs pas à pas
        // Move_Loop();

        switch (sequencer) {
            case 0:
                c = getchar_timeout_us(0);
				if (c >= 0) {
					Interp(c);
				}
                sequencer++;
                break;
            case 1:
                // Asserv_Loop();
                sequencer++;
                break;
            case 2:
                if(LD19.newScan){
                    LD19.newScan = 0;
                    has_data = true;
                }
                if(has_data){
                    RobotPose current_belief = Fusion_GetState();
                    RobotPose measured = Loc_ProcessScan(LD19.previousScan, &current_belief);
                    if (measured.valid){
                        Fusion_Correct(measured);
                    }

                    // LD19_printScanTeleplot(&LD19);
                }

                RobotPose final = Fusion_GetState();
                if(Timer_ms1 % 100 == 0){
                    printf(">robot:%d:%d|xy,clr\n", (int)final.x, (int)final.y);
                    printf(">room:0:0;1000:0;1000:2000;0:2000;0:0|xy,clr\n");
                }
                sequencer = 0;
                break;

            default:
                sequencer = 0;
                break;
        }
    }
    // cyw43_arch_deinit();
    return 0;
}

void Init_All(void)
{
    init_motors();
    Init_Asserv();
}

