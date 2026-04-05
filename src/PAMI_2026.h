#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/sync.h"

#include "mbedtls/sha1.h"
#include "mbedtls/base64.h"


#include <stdint.h>
#include <string.h>
#include <math.h>

#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/sync.h"


#include "PAMI_2026_IO.h"
#include "PAMI_Config.h"

#include "Timer.h"
#include "Interpreteur.h"
#include "BAU.h"

//include for led ws2812
#include "lib_ws2812/ws2812.h"

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

// include for pathfinding
#include "lib_avoid_obstacle/pathfinding.h"

// include for screen
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "lib_screen/GC9A01A.h"
#include "lib_screen/minion_eye.h"
#include "lib_screen/startup_screen.h"

// include for web socket
#include "tcp_server.h"
#include "tcp_client.h"
#include "websocket.h"
#include "Foxglove_web_socket.h"
#include "lwipopts.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "wifi_credentials.h"

#include "mbedtls/sha1.h"
#include "mbedtls/base64.h"

#define Abs_Ternaire(a)   (((a)<0)?(-a):(a))
#define Min_Ternaire(a,b) (((a)<(b))?(a):(b))
#define Max_Ternaire(a,b) (((a)>(b))?(a):(b))
#define sizetab(a) (sizeof(a)/sizeof(a[0]))


#define ROBOT_ID 2

extern int lidar_loc_en;

void Init_All(void);

uint8_t FREQ_Cmd(void);
void script_match(void);

extern LD19Instance LD19;