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

// Pin configuration
#define LCD_CS_PIN 21
#define LCD_DC_PIN 20
#define LCD_RST_PIN 22 // Use -1 if you skip the reset pin

// ==========================================
// --- Global Variables ---
// ==========================================
int lidar_loc_en = 1; // 0 = off, 1 = on (use lidar for localization correction)

int freq_robot_data_update = 20; // Hz
uint32_t last_robot_data_update_time = 0;

uint32_t last_batteries_update_time = 0;
float current_vbat = 0.0f; // Rendu global pour affichage écran et envoi TCP

int start_match = 0; // 1 = Match en cours
bool match_has_started_once = false; // Latch (verrou) pour l'affichage de l'écran

uint32_t last_interaction_time = 0;
uint32_t last_startup_draw_time = 0;

int match_state = 0;
int timer_match = 0;
int timer_match_delay = 85000; // 85 secondes (Coupe de France)
int timer_match_delay_endgame = 100000; 
Position Goal_Pos;

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
    Config_Load(); 

    // 2. Initialize Hardware & Peripherals
    Init_All();
    init_pathfinding_parameters();
    Fusion_Init(0.2f, 0.2f, 1.5f); // init x y theta

    // 3. Initialize Screen
    gc9a01a_t tft;
    gc9a01a_init(&tft, LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN);
    gc9a01a_begin(&tft); 
    minion_eye_init(&tft);

    // --- Initialisation du Wi-Fi ---
    bool wifi_initialized = false;
    bool wifi_connected = false;

    if (cyw43_arch_init() == 0) {
        wifi_initialized = true;
        cyw43_arch_enable_sta_mode();
        printf("\nRecherche de réseaux Wi-Fi...\n");

        for (int i = 0; i < num_wifi_networks; i++) {
            if (cyw43_arch_wifi_connect_timeout_ms(wifi_networks[i].ssid, wifi_networks[i].password, CYW43_AUTH_WPA2_AES_PSK, 10000) == 0) {
                printf(">> Wi-Fi connecté à '%s' !\n", wifi_networks[i].ssid);
                wifi_connected = true;
                break; 
            }
        }
        if (!wifi_connected) printf("\nAucun Wi-Fi trouvé. Mode STANDALONE.\n");
    } else {
        printf("Échec init Wi-Fi. Mode STANDALONE.\n");
    }

    if (wifi_connected) {
        tcp_state = tcp_server_open();
    }

    // --- Variables d'état GPIO ---
    bool last_leash_state = gpio_get(LEASH_PIN);
    bool last_au_state    = gpio_get(AU_PIN);
    bool last_team_state  = gpio_get(TEAM_PIN);
    
    bool team_state = last_team_state; // 0 = BLUE, 1 = YELLOW
    bool au_state   = last_au_state;   // 0 = normal mode, 1 = AU mode

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
        bool interaction_detected = false;

        // --- LECTURE DES GPIO ---
        bool current_leash_state = gpio_get(LEASH_PIN);
        if (current_leash_state != last_leash_state) {
            printf("LEASH : %s\n", current_leash_state ? "ACTIVE" : "INACTIVE");
            if(current_leash_state) {
                start_match = 1;
                match_has_started_once = true; // VERROU DÉFINITIF POUR L'ÉCRAN
                printf("MATCH STARTED\n");
            } else {
                start_match = 0; 
            }
            interaction_detected = true;
            last_leash_state = current_leash_state;
        }

        bool current_team_state = gpio_get(TEAM_PIN);
        if (current_team_state != last_team_state) {
            team_state = current_team_state ? 0 : 1;
            interaction_detected = true;
            last_team_state = current_team_state;
        }

        bool current_au_state = gpio_get(AU_PIN);
        if (current_au_state != last_au_state) {
            printf("AU : %d\n", current_au_state);
            interaction_detected = true;
            last_au_state = current_au_state;
        }

        // Si une interaction a eu lieu, on réarme le compteur de 20s
        if (interaction_detected && !match_has_started_once) {
            last_interaction_time = current_time;
        }

        // Sécurité Arrêt d'Urgence
        if (!current_au_state){
            motion_free();
            for (int i = 0; i < 3; i++) gpio_put(11, 1); 
        }

        if (wifi_connected && tcp_state != NULL) {
            cyw43_arch_poll();
        }

        // --- GESTION DE LA BATTERIE (Toutes les 1s) ---
        if (current_time - last_batteries_update_time >= 1000) {
            adc_select_input(0);
            float adc_val = (float)adc_read();
            current_vbat = (adc_val / 4095.0f) * 9.9f;

            // Envoi TCP
            if (tcp_state && tcp_state->is_connected && tcp_state->can_send) {
                char bat_msg[32];
                snprintf(bat_msg, sizeof(bat_msg), "BAT:%.2f\n", (double)current_vbat);
                char ws_buf[128];
                uint64_t pack_len = WS_BuildPacket(ws_buf, sizeof(ws_buf), WEBSOCKET_OPCODE_TEXT, bat_msg, strlen(bat_msg), 0);
                tcp_server_send_data(tcp_state, (uint8_t*)ws_buf, pack_len);
            }
            last_batteries_update_time = current_time;
        }

        // --- GESTION DE L'ÉCRAN ---
        if (match_has_started_once) {
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
                                    current_vbat, 
                                    actual_now.x * 1000.0f, // Conversion en mm
                                    actual_now.y * 1000.0f, 
                                    actual_now.theta, 
                                    team_state, 
                                    0); 
                last_startup_draw_time = current_time;
            }
        } 
        else {
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
                if (current_au_state == 1) { 
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

                if (current_au_state == 0) desired_led_state = 0; // AU -> Rouge
                else if (team_state == 0) desired_led_state = 1;  // BLUE
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
                if(current_au_state == 1){
                    script_match(); 
                }
                sequencer++;
                break;
            }
            case 5: {
                if (wifi_initialized) cyw43_arch_poll();
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
void Init_All(void)
{
    init_motors();
    Init_Asserv();

    gpio_init(LEASH_PIN);
    gpio_set_dir(LEASH_PIN, GPIO_IN);

    gpio_init(AU_PIN);
    gpio_set_dir(AU_PIN, GPIO_IN);
    gpio_pull_up(AU_PIN); // Ajouté depuis ton code d'origine

    gpio_init(TEAM_PIN);
    gpio_set_dir(TEAM_PIN, GPIO_IN);

    adc_init();
    adc_gpio_init(26);
}

uint8_t FREQ_Cmd(void) {
    uint32_t val32;
    if (Get_Param_u32(&val32)){
        return PARAM_ERROR_CODE;
    }
    freq_robot_data_update = (int)val32;
    return 0;
}

void script_match(void) {
    switch (match_state) {
        case 0:
            if (start_match) match_state++;
            break;
        case 1:
            Goal_Pos.x = 0.5;
            Goal_Pos.y = 1.0;
            Goal_Pos.t = 1.5708; 
            motion_pos(Goal_Pos);
            match_state++; 
            break;
        case 2:
            if (motion_done == 1) match_state++;
            break;
        case 3:
            Goal_Pos.x = 0.5;
            Goal_Pos.y = 0.2;
            Goal_Pos.t = 1.5708;
            motion_pos(Goal_Pos);
            match_state++;
            break;
        case 4:
            if (motion_done == 1) match_state = 1;
            break;
        default:
            break;
    }
}