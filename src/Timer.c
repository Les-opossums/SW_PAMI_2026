#include "PAMI_2026.h"

uint64_t Timer_us1 = 0; // Microsecondes
uint64_t Timer_ms1 = 0; // Millisecondes

void Timer_Update(void) {
    // Cette fonction doit être appelée toutes les 1 ms
    Timer_us1 = time_us_64();
    Timer_ms1 = Timer_us1 / 1000;
}