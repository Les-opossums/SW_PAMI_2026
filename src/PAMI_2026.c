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

int lidar_loc_en = 1; // 0 = off, 1 = on (use lidar for localization correction)

// Pin configuration
// #define LCD_CS_PIN 17
// #define LCD_DC_PIN 16
// #define LCD_RST_PIN 15 // Use -1 if you skip the reset pin

int freq_robot_data_update = 20; // Hz
int last_robot_data_update_time = 0;

int start_match = 0; // Set to 1 when the match starts (e.g., when the leash is activated)

// --- Helper: Ease-Out Interpolation ---
// Makes movement look organic (fast start, slow stop)
float ease_out_cubic(float t) {
    return 1.0f - powf(1.0f - t, 3.0f);
}

// ==========================================
// --- Global State LIDAR (HEAD) ---
// ==========================================
LD19Instance LD19;

void core1_entry() {
    LIDAR_UART_init();
    LD19_init(&LD19);
    LD19_enableFiltering(&LD19);
    LD19_setDistanceRange(&LD19, 100, 4000); // 0.1m to 4m
    while(1){
        LD19_readScan(&LD19, UART_ID);
    }
}

// ==========================================
// --- MAIN ---
// ==========================================
int main()
{
    // 1. Initialize standard I/O
    stdio_init_all();
    sleep_ms(2000); // wait for stdio to be ready

    // 2. Initialize Screen (Feature/Screen)
    // gc9a01a_t tft;
    // gc9a01a_init(&tft, LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN);
    // gc9a01a_begin(&tft); // Uses default SPI_DEFAULT_FREQ (40MHz)
    // minion_eye_init(&tft);

    // 3. Initialize Robot Logic (HEAD)
    int sequencer = 0;
    Fusion_Init(0.2f, 0.2f, 1.5f); // init x y theta
    Init_All();

    // --- Initialisation du Wi-Fi ---

    bool wifi_initialized = false;
    bool wifi_connected = false;

    if (cyw43_arch_init() == 0) {
        wifi_initialized = true;
        cyw43_arch_enable_sta_mode();

        printf("Connexion au Wi-Fi...\n");
        if (cyw43_arch_wifi_connect_timeout_ms("Opossum", "r28w3fr7j3zu8r4", CYW43_AUTH_WPA2_AES_PSK, 10000)) {
            printf("Échec de la connexion Wi-Fi. Le robot passe en mode STANDALONE.\n");
            // ON NE MET PLUS DE return 1; ICI !
        } else {
            printf("Wi-Fi connecté !\n");
            wifi_connected = true;
            extern cyw43_t cyw43_state;
            uint32_t ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
            printf("Adresse IP: %d.%d.%d.%d\n", 
                ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
        }
    } else {
        printf("Échec de l'initialisation de la puce Wi-Fi. Mode STANDALONE.\n");
    }

    // --- Démarrage du serveur TCP (Uniquement si connecté) ---
    tcp_server_t *tcp_state = NULL;
    if (wifi_connected) {
        tcp_state = tcp_server_open();
        if (!tcp_state) {
            printf("Erreur lors de l'ouverture du serveur TCP\n");
        } else {
            printf("Serveur TCP prêt sur le port %d\n", TCP_SERVER_PORT);
        }
    } else {
        printf("Serveur TCP ignoré (Mode Standalone).\n");
    }

    //lecture de la pin de la laisse
    bool last_leash_state = gpio_get(LEASH_PIN);
    bool current_leash_state = last_leash_state;

    bool last_au_state = gpio_get(AU_PIN);
    bool current_au_state = last_au_state;
    bool au_state = current_au_state; // 0 = normal mode, 1 = AU mode (no asserv, no LED update)

    bool last_team_state = gpio_get(TEAM_PIN);
    bool current_team_state = last_team_state;
    bool team_state = current_team_state; // 0 = BLUE, 1 = YELLOW

    led_rgb_init();

    Path_Init();
    // Launch Lidar on Core 1
    multicore_launch_core1(core1_entry);

    printf("PAMI-2026 ready (Lidar + Screen).\n");

    while (true) {
        // Lecture de l'état actuel du PIN de la laisse
        current_leash_state = gpio_get(LEASH_PIN);
        if (current_leash_state != last_leash_state) {
            // sleep_ms(50);
            // On affiche le message demandé
            printf("LEASH : %s\n", current_leash_state ? "ACTIVE" : "INACTIVE");
            if(current_leash_state) {
                start_match = 1; // Match starts when leash is activated
                printf("MATCH STARTED\n");
            } else {
                start_match = 0; // Match ends when leash is deactivated
            }
            last_leash_state = current_leash_state;
        }

        // Lecture de l'état actuel du PIN TEAM
        current_team_state = gpio_get(TEAM_PIN);
        if (current_team_state != last_team_state) {
            // sleep_ms(50);
            if(current_team_state) {
                team_state = 0; // Team BLUE
            } else {
                team_state = 1; // Team YELLOW
            }
            last_team_state = current_team_state;
        }

        // Lecture de l'état actuel du PIN AU
        current_au_state = gpio_get(AU_PIN);
        if (current_au_state != last_au_state) {
            // sleep_ms(50);
            // On affiche le message demandé 0 ou 1
            printf("AU : %d\n", current_au_state);
            last_au_state = current_au_state;
        }

        if (!current_au_state){
            // disable motors
            motion_free();
            for (int i = 0; i < 3; i++) {
                gpio_put(11, 1); // Disable power to stepper drivers
            }
        }

        // --- A. Screen Update ---
        // On met à jour l'animation à chaque tour de boucle
        // minion_eye_update_non_blocking();
        // --- B. Robot Logic ---
        bool has_data = false;
        Timer_Update(); // Met à jour les timers
        int c;

        // Met à jour les moteurs pas à pas
        Move_Loop();


        // Variables pour la gestion asynchrone du Wi-Fi
        uint32_t last_wifi_check_time = 0;
        bool wifi_reconnecting = false;

        switch (sequencer) {
            case 0:
                c = getchar_timeout_us(0);
                if (c >= 0) {
                    Interp(c);
                }
                sequencer++;
                break;
            case 1:
                if (current_au_state == 1) { // Only run asserv loop if not in AU mode
                    Asserv_Loop();
                    if (Timer_ms1 % freq_robot_data_update == 0 && Timer_ms1 != last_robot_data_update_time) { // e.g., 20 Hz
                        // printf("ROBOTDATA 1 2 3 4 5 6\n");

                        last_robot_data_update_time = Timer_ms1;
                    }
                }
                sequencer++;
                break;
            case 2:
                static RobotPose snapshot_pose; // La "photo" de l'odométrie

                if(LD19.newScan){
                    LD19.newScan = 0;
                    // 1. On prend une photo de la position EXACTE à la fin du scan Lidar
                    snapshot_pose = Fusion_GetState(); 
                    has_data = true;
                }

                if(has_data){
                    // 2. On aide la localisation avec notre photo (en mm)
                    RobotPose belief_for_loc = snapshot_pose;
                    belief_for_loc.x *= 1000.0f;
                    belief_for_loc.y *= 1000.0f;

                    // --- CALCUL LONG (~100ms) ---
                    // Pendant ce temps, l'asservissement continue de faire avancer le robot
                    RobotPose measured = Loc_ProcessScan(LD19.previousScan, &belief_for_loc);
                    
                    if (measured.valid && lidar_loc_en){
                        printf("LIDAR LOC: x=%.1f y=%.1f t=%.2f\n", measured.x, measured.y, measured.theta);

                        measured.x /= 1000.0f;
                        measured.y /= 1000.0f;
                        
                        // 3. CALCUL DE L'ERREUR DANS LE PASSÉ
                        // Quelle est la vraie erreur constatée par le Lidar à l'instant t-100ms ?
                        float err_x = measured.x - snapshot_pose.x;
                        float err_y = measured.y - snapshot_pose.y;
                        
                        float err_t = measured.theta - snapshot_pose.theta;
                        while (err_t < -M_PI) err_t += M_TWO_PI;
                        while (err_t >  M_PI) err_t -= M_TWO_PI;

                        // 4. PROJECTION DANS LE PRÉSENT
                        // On récupère la position actuelle (qui a avancé)
                        RobotPose actual_now = Fusion_GetState();
                        
                        // On crée une mesure "virtuelle" : ce que le Lidar mesurerait MAINTENANT
                        RobotPose projected_lidar = measured;
                        projected_lidar.x = actual_now.x + err_x;
                        projected_lidar.y = actual_now.y + err_y;
                        projected_lidar.theta = actual_now.theta + err_t;

                        // 5. APPEL DE TA FONCTION
                        // Ta fonction va calculer la diff entre projected_lidar et state (donc retomber sur nos err_x/err_y)
                        // et appliquer tes FUSION_GAIN et tes sécurités MAX_FUSION_JUMP !
                        Fusion_Correct(projected_lidar);
                    }
                }
                sequencer++;
                break;
            case 3: // led management
                // -1 = non initialisé, 0 = Rouge (AU), 1 = Bleu, 2 = Jaune
                static int current_led_state = -1; 
                int desired_led_state = 0;

                // 1. On détermine la couleur que l'on veut afficher
                if (current_au_state == 0) { 
                    desired_led_state = 0; // Mode Arrêt d'Urgence -> Rouge
                } else {
                    if (team_state == 0) {
                        desired_led_state = 1; // Team BLUE -> Bleu
                    } else {
                        desired_led_state = 2; // Team YELLOW -> Jaune
                    }
                }

                // 2. On n'envoie la commande QUE si la situation a changé
                if (desired_led_state != current_led_state) {
                    if (desired_led_state == 0) {
                        led_rgb_set_color(100, 0, 0);   // Rouge
                    } else if (desired_led_state == 1) {
                        led_rgb_set_color(0, 0, 100);   // Bleu
                    } else if (desired_led_state == 2) {
                        led_rgb_set_color(100, 100, 0); // Jaune
                    }
                    
                    // On met à jour la mémoire
                    current_led_state = desired_led_state;
                }

                sequencer++;
                break;
            case 4:
                if(current_au_state == 1){
                    script_match(); // Gère la logique de déplacement pendant le match
                }
                sequencer++;
                break;
            case 5:
                // On s'assure que la puce est bien initialisée avant de lui parler
                if (wifi_initialized) {
                    cyw43_arch_poll();
                }
                sequencer++;
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

    //init laisse
    gpio_init(LEASH_PIN);
    gpio_set_dir(LEASH_PIN, GPIO_IN);

    // init AU
    gpio_init(AU_PIN);
    gpio_set_dir(AU_PIN, GPIO_IN);

    //init TEAM
    gpio_init(TEAM_PIN);
    gpio_set_dir(TEAM_PIN, GPIO_IN);
}

uint8_t FREQ_Cmd(void) {
    uint32_t val32;
    if (Get_Param_u32(&val32)){
        return PARAM_ERROR_CODE;
    }
    freq_robot_data_update = (int)val32;
    return 0;
}



int match_state = 0;
int timer_match = 0;
int timer_match_delay = 5000; // ms à changer en 85000 pour la vraie durée d'un match
int timer_match_delay_endgame = 100000; // ms 
Position Goal_Pos;


void script_match(void) {
    switch (match_state) {
        case 0:
            if (start_match) {
                match_state++; // Go !
            }
            break;

        case 1: // ACTION : Lancement du premier mouvement
            Goal_Pos.x = 0.5;
            Goal_Pos.y = 1.0;
            Goal_Pos.t = 1.5708; // 90 degrés en radians
            motion_pos(Goal_Pos);
            match_state++; // On passe tout de suite à l'état d'attente
            break;
            
        case 2: // ATTENTE : On boucle ici tant que le robot roule
            if (motion_done == 1) {
                match_state++; // On passe au mouvement suivant
            }
            break;

        // --- MOUVEMENT 2 ---
        case 3: // ACTION : Demi-tour
            Goal_Pos.x = 0.5;
            Goal_Pos.y = 0.2;
            Goal_Pos.t = 1.5708;
            motion_pos(Goal_Pos);
            match_state++;
            break;

        case 4: // ATTENTE
            if (motion_done == 1) {
                match_state = 1;
            }
            break;

        default:
            // Match terminé ou attente de la fin des 90 secondes
            break;
    }
}