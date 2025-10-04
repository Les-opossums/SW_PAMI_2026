#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "hardware/uart.h"


#include "PAMI_2026_IO.h"

#include "Timer.h"
#include "LIDAR_UART.h"
#include "Interpreteur.h"

#include "STEPPER.h"

#define Abs_Ternaire(a)   (((a)<0)?(-a):(a))
#define Min_Ternaire(a,b) (((a)<(b))?(a):(b))
#define Max_Ternaire(a,b) (((a)>(b))?(a):(b))
#define sizetab(a) (sizeof(a)/sizeof(a[0]))

void Init_All(void);