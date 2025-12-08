#include "PAMI_2026.h"


LD19Instance LD19;

int main()
{
    stdio_init_all();
    
    int sequencer = 0;

    // Init_All();

    // Initialisation du LIDAR LD19
    LIDAR_UART_init();
    LD19_init(&LD19);
    LD19_enableFiltering(&LD19);

    printf("PAMI-2026 ready.\n");


    while (true) {
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
                // Lire les données du LIDAR
                if (LD19_readScan(&LD19, UART_ID)) {
                    if (LD19_isNewScan(&LD19)) {
                        // Nouvelle trame complète reçue
                        // Traiter les données
                        // Par exemple, afficher le scan
                        LD19_printScanTeleplot(&LD19);
                    }
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

