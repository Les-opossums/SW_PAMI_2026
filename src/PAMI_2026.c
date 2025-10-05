#include "PAMI_2026.h"

int main()
{
    stdio_init_all();
    
    int sequencer = 0;

    Init_All();

    printf("PAMI-2026 ready.\n");

    while (true) {
        Timer_Update(); // Met à jour les timers

        int c;

        // Met à jour les moteurs pas à pas
        Move_Loop();

        switch (sequencer) {
            case 0:
                c = getchar_timeout_us(0);
				if (c >= 0) {
					Interp(c);
				}
                sequencer++;
                break;
            case 1:
                Asserv_Loop();
                sequencer++;
                break;

            default:
                sequencer = 0;
                break;
        }
    }
}

void Init_All(void)
{
    init_motors();
    Init_Asserv();
}