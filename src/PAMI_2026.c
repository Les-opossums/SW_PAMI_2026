#include "PAMI_2026.h"

int main()
{
    stdio_init_all();
    
    int sequencer = 0;

    Init_All();

    while (true) {
        Timer_Update(); // Met à jour les timers

        int c;

        switch (sequencer) {
            case 0:
                c = getchar_timeout_us(0);
				if (c >= 0) {
					Interp(c);
				}
                sequencer++;
                break;
            case 1:
                Move_Loop();
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
}