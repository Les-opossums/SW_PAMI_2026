#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/sync.h"

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "hardware/uart.h"


#include "PAMI_2026_IO.h"

#include "Timer.h"
#include "Interpreteur.h"
#include "BAU.h"

//include for LD19 LIDAR
#include "lib_lidar/LIDAR_LD19.h"
#include "lib_lidar/LIDAR_UART.h"

// include for localization
#include "lib_localization/localization.h"
#include "lib_localization/fusion.h"

// include for move
#include "STEPPER.h"
#include "lib_asserv/Lib_Asserv.h"
#include "Asserv_Loop.h"
#include "Cmd_For_Move.h"


// include for web socket
#include "tcp_server.h"
#include "tcp_client.h"
#include "websocket.h"
#include "Foxglove_web_socket.h"
#include "lwipopts.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "mbedtls/sha1.h"
#include "mbedtls/base64.h"

#define Abs_Ternaire(a)   (((a)<0)?(-a):(a))
#define Min_Ternaire(a,b) (((a)<(b))?(a):(b))
#define Max_Ternaire(a,b) (((a)>(b))?(a):(b))
#define sizetab(a) (sizeof(a)/sizeof(a[0]))

void Init_All(void);