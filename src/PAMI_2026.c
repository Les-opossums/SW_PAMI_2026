#include "PAMI_2026.h"

#ifndef M_TWO_PI
#define M_TWO_PI 6.28318530717958647692f
#endif

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

// ==========================================
// --- Lidar TCP & Telemetry ---
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
// --- Global State LIDAR (HEAD) ---
// ==========================================
LD19Instance LD19;

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

    printf("LIDAR thread started on Core 1\n");

    while(1){
        LD19_readScan(&LD19, UART_ID);
    }
}


// ==========================================
// --- MAIN ---
// ==========================================
int main()
{
    // 1. Initialize standard I/O & Config
    stdio_init_all();
    sleep_ms(2000); 

    // 2. Load Config from Flash (ID, etc.)
    Config_Load(); 

    // 3. Initialize Hardware & Peripherals
    init_motors();
    Init_Asserv();

    IHM_init();

    init_pathfinding_parameters();
    Fusion_Init(0.2f, 0.2f, 1.5f); // init x y theta

    // 4. Initialize Screen
    gc9a01a_t tft;
    gc9a01a_init(&tft, LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN);
    gc9a01a_begin(&tft); 
    minion_eye_init(&tft);

    // --- Variables d'état GPIO ---
    bool last_leash_state = gpio_get(LEASH_PIN);
    bool last_au_state    = gpio_get(AU_PIN);
    bool last_team_state  = gpio_get(TEAM_PIN);
    
    bool team_state = last_team_state; // 0 = BLUE, 1 = YELLOW
    bool au_state   = last_au_state;   // 0 = normal mode, 1 = AU mode

    RobotPose pose_init = Fusion_GetState();

    IHM_get_battery_voltage(); // Lecture initiale de la batterie pour affichage dès le départ

     // Affiche l'écran de démarrage tant que le match n'a pas commencé
    startup_screen_show(&tft, 
                        current_config.pami_id, 
                        IHM.current_vbat,
                        pose_init.x,
                        pose_init.y, 
                        pose_init.theta, 
                        team_state, 
                        !au_state, 
                        false,
                        0); // Écran de démarrage initial

    sleep_ms(1000); 

    // --- Initialisation du Wi-Fi (ASYNCHRONE ET RETENTATIVES) ---
    bool wifi_initialized = false;
    bool wifi_connected = false;
    int current_wifi_index = 0;
    bool wifi_connection_in_progress = false;
    
    uint32_t wifi_start_time = 0;      // Pour le timeout de 10s d'un essai
    uint32_t last_wifi_check_time = 0; // Pour ne pas interroger la puce à chaque microseconde
    uint32_t last_wifi_retry_time = 0; // Pour la pause de 15s avant de recommencer toute la liste

    if (cyw43_arch_init() == 0) {
        wifi_initialized = true;
        cyw43_arch_enable_sta_mode();
        printf("\nRecherche de réseaux Wi-Fi...\n");

        if (num_wifi_networks > 0) {
            printf("Essai 1/%d : Tentative de connexion a '%s'...\n", num_wifi_networks, wifi_networks[0].ssid);
            cyw43_arch_wifi_connect_async(wifi_networks[0].ssid, wifi_networks[0].password, CYW43_AUTH_WPA2_AES_PSK);
            wifi_connection_in_progress = true;
            wifi_start_time = time_us_32() / 1000;
        } else {
            printf("Aucun réseau configuré. Mode STANDALONE (pas de retentatives).\n");
        }
    } else {
        printf("Échec init Wi-Fi. Mode STANDALONE définitif.\n");
    }

    led_rgb_init();
    Path_Init();

    // Lancement du LIDAR
    multicore_launch_core1(core1_entry);
    printf("PAMI-2026 ready.\n");

    // Initialise le timer d'interaction pour afficher l'écran de config au démarrage
    last_interaction_time = time_us_32() / 1000; 

    int sequencer = 0;

    // ==========================================
    // --- MAIN LOOP ---
    // ==========================================
    while (true) {
        Timer_Update();
        uint32_t current_time = Timer_ms1; 

        // --- GESTION DES BOUTONS & INTERACTIONS ---
        IHM_get_AU_state();
        IHM_get_team_state();
        IHM_get_leash_state();
        // Si une interaction a eu lieu, on réarme le compteur de 20s
        if (IHM.interaction_detected && !IHM.match_started_once) {
            last_interaction_time = current_time;
            IHM.interaction_detected = false;
        }

        // Sécurité Arrêt d'Urgence
        if (!IHM.au_state) {
            motion_free();
            for (int i = 0; i < 3; i++) gpio_put(11, 1); 
        }

        // --- GESTION DE LA BATTERIE (Toutes les 1s) ---
        if (current_time - last_batteries_update_time >= 1000) {
            IHM_get_battery_voltage();

            // Envoi TCP
            if (tcp_state && tcp_state->is_connected && tcp_state->can_send) {
                char bat_msg[32];
                snprintf(bat_msg, sizeof(bat_msg), "BAT:%.2f\n", (double)IHM.current_vbat);
                char ws_buf[128];
                uint64_t pack_len = WS_BuildPacket(ws_buf, sizeof(ws_buf), WEBSOCKET_OPCODE_TEXT, bat_msg, strlen(bat_msg), 0);
                tcp_server_send_data(tcp_state, (uint8_t*)ws_buf, pack_len);
            }
            last_batteries_update_time = current_time;
        }

        // --- GESTION DE L'ÉCRAN ---
        if (IHM.match_started_once) {
            // MATCH COMMENCÉ : Toujours le minion
            minion_eye_update_non_blocking();
        } 
        else if ((current_time - last_interaction_time) <= 20000) {
            // INTERACTION RÉCENTE (< 20s) : Écran de démarrage (mise à jour à 2Hz)
            if (current_time - last_startup_draw_time >= 500) {
                RobotPose actual_now = Fusion_GetState();
                // 0 en dernier argument pour ne pas bloquer la boucle
                startup_screen_show(&tft, 
                                    current_config.pami_id, // Affiche le véritable ID du robot
                                    IHM.current_vbat, 
                                    actual_now.x,
                                    actual_now.y, 
                                    actual_now.theta, 
                                    IHM.team_state,
                                    !IHM.au_state, // Affiche l'état d'urgence (rouge si AU actif)
                                    wifi_connected, // Affiche l'état du Wi-Fi 
                                    0); 
                last_startup_draw_time = current_time;
            }
        } else {
            // REPOS (> 20s) : Animation minion
            minion_eye_update_non_blocking();
        }


        // --- LOGIQUE ROBOT (Moteurs & Sequencer) ---
        Move_Loop();

        switch (sequencer) {
            case 0: {
                int c = getchar_timeout_us(0);
                if (c >= 0) Interp(c);
                sequencer++;
                break;
            }
            case 1: {
                if (IHM.au_state == 1) { 
                    Asserv_Loop();
                }
                sequencer++;
                break;
            }
            case 2: {
                static RobotPose snapshot_pose;
                static RobotPose last_lidar_pose = {0.0f, 0.0f, 0.0f, true}; 

                if(LD19.newScan){
                    LD19.newScan = 0; 
                    snapshot_pose = Fusion_GetState(); 
                    
                    RobotPose belief_for_loc = snapshot_pose;
                    belief_for_loc.x *= 1000.0f;
                    belief_for_loc.y *= 1000.0f;

                    RobotPose measured = Loc_ProcessScan(LD19.previousScan, &belief_for_loc);
                    
                    if (measured.valid && lidar_loc_en){
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
                    }

                    if (wifi_connected && tcp_state != NULL) {
                        send_lidar_telemetry(tcp_state, last_lidar_pose.x, last_lidar_pose.y, last_lidar_pose.theta);
                    }
                }
                sequencer++;
                break;
            }
            case 3: { // led management
                static int current_led_state = -1; 
                int desired_led_state = 0;

                if (IHM.au_state == 0) desired_led_state = 0; // AU -> Rouge
                else if (IHM.team_state == 0) desired_led_state = 1;  // BLUE
                else desired_led_state = 2;                       // YELLOW

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
                if(IHM.au_state == 1){
                    script_loop(); 
                }
                sequencer++;
                break;
            }
            case 5: {
                if (wifi_initialized) {
                    cyw43_arch_poll(); // Nécessaire pour maintenir la liaison asynchrone

                    // On évalue l'état réseau toutes les 500 ms (pour ne pas saturer)
                    if (current_time - last_wifi_check_time >= 500) {
                        last_wifi_check_time = current_time;
                        int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

                        // --- ETAT 1 : TENTATIVE DE CONNEXION EN COURS ---
                        if (wifi_connection_in_progress) {
                            if (link_status == CYW43_LINK_UP) {
                                printf(">> Wi-Fi connecté avec succès à '%s' !\n", wifi_networks[current_wifi_index].ssid);
                                wifi_connected = true;
                                wifi_connection_in_progress = false;
                                
                                uint32_t ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
                                printf(">> Adresse IP : %d.%d.%d.%d\n", 
                                    ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
                                
                                // On ouvre le TCP si ce n'est pas déjà fait
                                if (tcp_state == NULL) {
                                    tcp_state = tcp_server_open();
                                }
                            } 
                            else if (link_status < 0 || (current_time - wifi_start_time > 10000)) {
                                printf("Échec ou timeout (10s) pour '%s'.\n", wifi_networks[current_wifi_index].ssid);
                                current_wifi_index++;
                                
                                if (current_wifi_index < num_wifi_networks) {
                                    printf("Essai %d/%d : Tentative de connexion a '%s'...\n", 
                                           current_wifi_index + 1, num_wifi_networks, wifi_networks[current_wifi_index].ssid);
                                    cyw43_arch_wifi_connect_async(wifi_networks[current_wifi_index].ssid, wifi_networks[current_wifi_index].password, CYW43_AUTH_WPA2_AES_PSK);
                                    wifi_start_time = current_time;
                                } else {
                                    printf("\nAucun Wi-Fi trouvé sur cette passe. Le robot passe en veille réseau.\n");
                                    wifi_connection_in_progress = false;
                                    last_wifi_retry_time = current_time; // Début de la période de repos
                                }
                            }
                        } 
                        // --- ETAT 2 : CONNECTÉ (Surveillance de coupure) ---
                        else if (wifi_connected) {
                            if (link_status != CYW43_LINK_UP) {
                                printf("\n[ALERTE] Perte de la connexion Wi-Fi ! Passage en veille réseau.\n");
                                wifi_connected = false;
                                last_wifi_retry_time = current_time; // On attend avant de bourriner
                            }
                        } 
                        // --- ETAT 3 : DÉCONNECTÉ ET EN PAUSE ---
                        else {
                            // On retente toute la liste des Wi-Fi toutes les 15 secondes
                            if (current_time - last_wifi_retry_time > 15000 && num_wifi_networks > 0) {
                                printf("\nNouvelle tentative de recherche de réseaux Wi-Fi...\n");
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