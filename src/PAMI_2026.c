#include "PAMI_2026.h"

#ifndef M_TWO_PI
#define M_TWO_PI 6.28318530717958647692f
#endif

// #define DEBUG_TIMING_CORE1 
// #define DEBUG_TIMING_CORE0
// ==========================================
// --- Configuration SCREEN (Feature/Screen) ---
// ==========================================
#define CENTER_X 120
#define CENTER_Y 120
#define MAX_PUPIL_DIST 35  // How far the eye can look from center
#define BLINK_CLOSE_Y 110  // How far eyelids close (120 = fully closed)

// ==========================================
// --- Global Variables ---
// ==========================================
int lidar_loc_en = 1; // 0 = off, 1 = on (use lidar for localization correction)

int freq_robot_data_update = 20; // Hz
uint32_t last_robot_data_update_time = 0;
uint32_t last_batteries_update_time = 0;
uint32_t last_interaction_time = 0;
uint32_t last_startup_draw_time = 0;

// --- Variables partagées (Core 1 -> Core 0) pour la Télémétrie TCP ---
volatile bool shared_new_tel_ready = false;
volatile float shared_tel_x = 0.0f;
volatile float shared_tel_y = 0.0f;
volatile float shared_tel_theta = 0.0f;

// ==========================================
// --- Lidar TCP & Telemetry (Géré par Core 0) ---
// ==========================================
volatile uint8_t enable_tcp_telemetry = 1; 
tcp_server_t *tcp_state = NULL;

#pragma pack(push, 1)
typedef struct {
    uint8_t magic[2];     // 0xAA, 0xBB (Header de synchronisation)
    float robot_x;        // Position X calculée (m)
    float robot_y;        // Position Y calculée (m)
    float robot_theta;    // Angle Theta (rad)
    uint16_t num_points;  // Nombre de points Lidar dans cette trame
} tcp_lidar_frame_header_t;
#pragma pack(pop)

void send_lidar_telemetry(tcp_server_t *tcp_state, float rx, float ry, float rtheta) {
    if (!enable_tcp_telemetry || !tcp_state || !tcp_state->is_connected || !tcp_state->can_send) return; 

    uint16_t num_points = LD19.previousScan->index;
    size_t points_size = num_points * 2 * sizeof(float);
    size_t total_size = sizeof(tcp_lidar_frame_header_t) + points_size;

    if (tcp_sndbuf(tcp_state->client_pcb) < total_size) return;

    static uint8_t tx_buffer[4096]; 
    if (total_size > sizeof(tx_buffer)) return;

    tcp_lidar_frame_header_t *header = (tcp_lidar_frame_header_t *)tx_buffer;
    header->magic[0] = 0xAA;
    header->magic[1] = 0xBB;
    header->robot_x = rx; 
    header->robot_y = ry;
    header->robot_theta = rtheta;
    header->num_points = num_points;

    float *points_data = (float *)(tx_buffer + sizeof(tcp_lidar_frame_header_t));
    for (uint16_t i = 0; i < num_points; i++) {
        points_data[2*i]     = LD19.previousScan->points[i].x;
        points_data[2*i + 1] = LD19.previousScan->points[i].y;
    }

    static uint8_t ws_final_buffer[8192]; 
    uint64_t ws_packet_len = WS_BuildPacket((char*)ws_final_buffer, sizeof(ws_final_buffer), 
                                            WEBSOCKET_OPCODE_BIN, 
                                            (char*)tx_buffer, total_size, 0);

    if (ws_packet_len > 0 && ws_packet_len < sizeof(ws_final_buffer)) {
        tcp_server_send_data(tcp_state, ws_final_buffer, ws_packet_len);
    }
}

// --- Helper: Ease-Out Interpolation ---
float ease_out_cubic(float t) {
    return 1.0f - powf(1.0f - t, 3.0f);
}

// ==========================================
// --- Global State LIDAR & SYSTEM (HEAD) ---
// ==========================================
LD19Instance LD19;

// ==========================================
// --- CORE 1 : TEMPS RÉEL (Asserv + Lidar) ---
// ==========================================
void core1_entry() {
    multicore_lockout_victim_init(); // Autorise pause pendant écriture Flash ID

    LIDAR_UART_init();
    LD19_init(&LD19);
    LD19_enableFiltering(&LD19);
    LD19_setDistanceRange(&LD19, 100, 4000); // 0.1m to 4m

    // blindspots
    LD19_addBlindSpot(&LD19, 60.0f, 12.0f);
    LD19_addBlindSpot(&LD19, 180.0f, 12.0f);
    LD19_addBlindSpot(&LD19, 240.0f, 12.0f);

    printf("LIDAR & ASSERV thread started on Core 1\n");

    // Variables pour le Lidar
    static RobotPose snapshot_pose;
    static RobotPose last_lidar_pose = {0.0f, 0.0f, 0.0f, true}; 

    // Profilage CPU Core 1
    uint32_t profilage_start_timer = time_us_32() / 1000; 
    uint32_t loop_time_max_us = 0;
    uint32_t loop_time_sum_us = 0;
    uint32_t loop_count = 0;

    while(1) {
        // [PROFILAGE CORE 1] Top chrono
        uint32_t loop_start_us = time_us_32();

        // 1. LECTURE LIDAR (Non-bloquant)
        LD19_readScan(&LD19, UART_ID);

        // 2. ASSERVISSEMENT & MOTEURS
        Move_Loop();
        if (IHM.au_state == 1) { 
            Asserv_Loop();
        }

        // 3. TRAITEMENT LIDAR & FUSION
        if(LD19.newScan) {
            LD19.newScan = 0; 
            snapshot_pose = Fusion_GetState(); 
            
            RobotPose belief_for_loc = snapshot_pose;
            belief_for_loc.x *= 1000.0f;
            belief_for_loc.y *= 1000.0f;

            // Calcul lourd de la localisation
            RobotPose measured = Loc_ProcessScan(LD19.previousScan, &belief_for_loc);
            
            if (measured.valid && lidar_loc_en) {
                last_lidar_pose = measured;
                measured.x /= 1000.0f;
                measured.y /= 1000.0f;
                
                float err_x = measured.x - snapshot_pose.x;
                float err_y = measured.y - snapshot_pose.y;
                
                float err_t = measured.theta - snapshot_pose.theta;
                while (err_t < -M_PI) err_t += M_TWO_PI;
                while (err_t >  M_PI) err_t -= M_TWO_PI;

                RobotPose actual_now = Fusion_GetState();
                RobotPose projected_lidar = measured;
                projected_lidar.x = actual_now.x + err_x;
                projected_lidar.y = actual_now.y + err_y;
                projected_lidar.theta = actual_now.theta + err_t;

                Fusion_Correct(projected_lidar);

                // On passe les données au Core 0 pour l'envoi Wi-Fi
                shared_tel_x = last_lidar_pose.x;
                shared_tel_y = last_lidar_pose.y;
                shared_tel_theta = last_lidar_pose.theta;
                shared_new_tel_ready = true;
            }
        }

        #ifdef DEBUG_TIMING_CORE1
            // [PROFILAGE CORE 1] Fin du chrono
            uint32_t loop_end_us = time_us_32();
            uint32_t current_loop_time = loop_end_us - loop_start_us;

            if (current_loop_time > loop_time_max_us) loop_time_max_us = current_loop_time;
            loop_time_sum_us += current_loop_time;
            loop_count++;

            uint32_t current_time = time_us_32() / 1000;
            if (current_time - profilage_start_timer >= 1000) {
                if (loop_count > 0) {
                    uint32_t loop_time_avg_us = loop_time_sum_us / loop_count;
                    printf("[CPU Core 1] Freq: %lu boucles/sec | Boucle (moy): %lu us | Boucle (max): %lu us\n", 
                        loop_count, loop_time_avg_us, loop_time_max_us);
                }
                loop_time_max_us = 0; loop_time_sum_us = 0; loop_count = 0;
                profilage_start_timer = current_time;
            }
        #endif
    }
}

// ==========================================
// --- MAIN (CORE 0) : IHM & GESTION RÉSEAU ---
// ==========================================
int main()
{
    // 1. Initialize standard I/O & Config
    stdio_init_all();
    sleep_ms(2000); 

    // 2. Load Config from Flash
    Config_Load(); 

    // 3. Initialize Hardware & Peripherals (Asserv & Moteurs)
    init_motors();
    Init_Asserv();
    IHM_init();
    init_pathfinding_parameters();
    Fusion_Init(0.2f, 0.2f, 1.5f);

    // 4. Initialize Screen
    gc9a01a_t tft;
    gc9a01a_init(&tft, LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN);
    gc9a01a_begin(&tft); 
    minion_eye_init(&tft);

    // 5. Init servo
    init_pami_servo();

    // Variables IHM
    bool last_leash_state = gpio_get(LEASH_PIN);
    bool last_au_state    = gpio_get(AU_PIN);
    bool last_team_state  = gpio_get(TEAM_PIN);
    bool team_state = last_team_state; 
    bool au_state   = last_au_state;   

    RobotPose pose_init = Fusion_GetState();
    IHM_get_battery_voltage(); 

    startup_screen_show(&tft, current_config.pami_id, IHM.current_vbat, pose_init.x, pose_init.y, pose_init.theta, team_state, !au_state, false, 0); 
    sleep_ms(1000); 

    // --- Wi-Fi Asynchrone ---
    bool wifi_initialized = false;
    bool wifi_connected = false;
    int current_wifi_index = 0;
    bool wifi_connection_in_progress = false;
    uint32_t wifi_start_time = 0;      
    uint32_t last_wifi_check_time = 0; 
    uint32_t last_wifi_retry_time = 0; 

    if (cyw43_arch_init() == 0) {
        wifi_initialized = true;
        cyw43_arch_enable_sta_mode();
        printf("\nRecherche de réseaux Wi-Fi...\n");

        if (num_wifi_networks > 0) {
            cyw43_arch_wifi_connect_async(wifi_networks[0].ssid, wifi_networks[0].password, CYW43_AUTH_WPA2_AES_PSK);
            wifi_connection_in_progress = true;
            wifi_start_time = time_us_32() / 1000;
        }
    } else {
        printf("Échec init Wi-Fi. Mode STANDALONE.\n");
    }

    led_rgb_init();
    Path_Init();

    // DÉMARRAGE DU CORE 1 (Asserv & Lidar partent !)
    multicore_launch_core1(core1_entry);
    printf("PAMI-2026 ready.\n");

    last_interaction_time = time_us_32() / 1000; 
    int sequencer = 0;

    // Profilage CPU Core 0
    uint32_t profilage_start_timer = time_us_32() / 1000;
    uint32_t loop_time_max_us = 0;
    uint32_t loop_time_sum_us = 0;
    uint32_t loop_count = 0;

    // ==========================================
    // --- MAIN LOOP (CORE 0) ---
    // ==========================================
    while (true) {
        uint32_t loop_start_us = time_us_32();

        Timer_Update(); // Met à jour Timer_ms1 pour tout le système
        uint32_t current_time = Timer_ms1; 

        // --- GESTION IHM ---
        IHM_get_AU_state();
        IHM_get_team_state();
        IHM_get_leash_state();
        
        if (IHM.interaction_detected && !IHM.match_started_once) {
            last_interaction_time = current_time;
            IHM.interaction_detected = false;
        }

        // Sécurité AU (Le Core 1 coupe l'asserv, mais le Core 0 fige les roues par précaution)
        if (!IHM.au_state) {
            motion_free();
            for (int i = 0; i < 3; i++) gpio_put(11, 1); 
        }

        // --- GESTION BATTERIE ---
        if (current_time - last_batteries_update_time >= 1000) {
            IHM_get_battery_voltage();
            if (tcp_state && tcp_state->is_connected && tcp_state->can_send) {
                char bat_msg[32];
                snprintf(bat_msg, sizeof(bat_msg), "BAT:%.2f\n", (double)IHM.current_vbat);
                char ws_buf[128];
                uint64_t pack_len = WS_BuildPacket(ws_buf, sizeof(ws_buf), WEBSOCKET_OPCODE_TEXT, bat_msg, strlen(bat_msg), 0);
                tcp_server_send_data(tcp_state, (uint8_t*)ws_buf, pack_len);
            }
            last_batteries_update_time = current_time;
        }

        // --- GESTION ÉCRAN ---
        if (IHM.match_started_once) {
            minion_eye_update_non_blocking();
        } 
        else if ((current_time - last_interaction_time) <= 20000) {
            if (current_time - last_startup_draw_time >= 500) {
                RobotPose actual_now = Fusion_GetState();
                startup_screen_show(&tft, current_config.pami_id, IHM.current_vbat, actual_now.x, actual_now.y, actual_now.theta, IHM.team_state, !IHM.au_state, wifi_connected, 0); 
                last_startup_draw_time = current_time;
            }
        } else {
            minion_eye_update_non_blocking();
        }

        // --- SÉQUENCEUR (Core 0) ---
        switch (sequencer) {
            case 0: {
                int c = getchar_timeout_us(0);
                if (c >= 0) Interp(c);
                sequencer++;
                break;
            }
            case 1: {
                // (Asserv_Loop a été déplacé sur le Core 1)
                sequencer++;
                break;
            }
            case 2: {
                // Télémétrie : On récupère les données calculées par le Core 1
                if (shared_new_tel_ready) {
                    if (wifi_connected && tcp_state != NULL) {
                        send_lidar_telemetry(tcp_state, shared_tel_x, shared_tel_y, shared_tel_theta);
                    }
                    shared_new_tel_ready = false;
                }
                sequencer++;
                break;
            }
            case 3: { 
                // Gestion Leds
                static int current_led_state = -1; 
                int desired_led_state = 0;

                if (IHM.au_state == 0) desired_led_state = 0; 
                else if (IHM.team_state == 0) desired_led_state = 1;  
                else desired_led_state = 2;                       

                if (desired_led_state != current_led_state) {
                    if (desired_led_state == 0) led_rgb_set_color(100, 0, 0);   
                    else if (desired_led_state == 1) led_rgb_set_color(0, 0, 100);  
                    else if (desired_led_state == 2) led_rgb_set_color(100, 100, 0);
                    current_led_state = desired_led_state;
                }
                sequencer++;
                break;
            }
            case 4: {
                script_loop(); 
                sequencer++;
                break;
            }
            case 5: {
                // Gestion Wi-Fi (Core 0 gère le réseau)
                if (wifi_initialized) {
                    cyw43_arch_poll(); 

                    if (current_time - last_wifi_check_time >= 500) {
                        last_wifi_check_time = current_time;
                        int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

                        if (wifi_connection_in_progress) {
                            if (link_status == CYW43_LINK_UP) {
                                wifi_connected = true;
                                wifi_connection_in_progress = false;
                                if (tcp_state == NULL) tcp_state = tcp_server_open();
                            } 
                            else if (link_status < 0 || (current_time - wifi_start_time > 10000)) {
                                current_wifi_index++;
                                if (current_wifi_index < num_wifi_networks) {
                                    cyw43_arch_wifi_connect_async(wifi_networks[current_wifi_index].ssid, wifi_networks[current_wifi_index].password, CYW43_AUTH_WPA2_AES_PSK);
                                    wifi_start_time = current_time;
                                } else {
                                    wifi_connection_in_progress = false;
                                    last_wifi_retry_time = current_time;
                                }
                            }
                        } 
                        else if (wifi_connected) {
                            if (link_status != CYW43_LINK_UP) {
                                wifi_connected = false;
                                last_wifi_retry_time = current_time; 
                            }
                        } 
                        else {
                            if (current_time - last_wifi_retry_time > 15000 && num_wifi_networks > 0) {
                                current_wifi_index = 0;
                                cyw43_arch_wifi_connect_async(wifi_networks[0].ssid, wifi_networks[0].password, CYW43_AUTH_WPA2_AES_PSK);
                                wifi_connection_in_progress = true;
                                wifi_start_time = current_time;
                            }
                        }
                    }
                }
                sequencer++;
                break;
            }
            default:
                sequencer = 0;
                break;
        }

        #ifdef DEBUG_TIMING_CORE0
            // [PROFILAGE CORE 0]
            uint32_t loop_end_us = time_us_32();
            uint32_t current_loop_time = loop_end_us - loop_start_us;

            if (current_loop_time > loop_time_max_us) loop_time_max_us = current_loop_time;
            loop_time_sum_us += current_loop_time;
            loop_count++;

            if (current_time - profilage_start_timer >= 1000) {
                if (loop_count > 0) {
                    uint32_t loop_time_avg_us = loop_time_sum_us / loop_count;
                    printf("[CPU Core 0] Freq: %lu boucles/sec | Boucle (moy): %lu us | Boucle (max): %lu us\n", 
                        loop_count, loop_time_avg_us, loop_time_max_us);
                }
                loop_time_max_us = 0; loop_time_sum_us = 0; loop_count = 0;
                profilage_start_timer = current_time;
            }
        #endif
    }
    return 0;
}

// ==========================================
// --- Additional Functions ---
// ==========================================
uint8_t FREQ_Cmd(void) {
    uint32_t val32;
    if (Get_Param_u32(&val32)){
        return PARAM_ERROR_CODE;
    }
    freq_robot_data_update = (int)val32;
    return 0;
}