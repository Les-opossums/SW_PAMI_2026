#include "PAMI_2026.h"

// ==========================================
// --- Configuration SCREEN (Feature/Screen) ---
// ==========================================
#define CENTER_X 120
#define CENTER_Y 120
#define MAX_PUPIL_DIST 35  // How far the eye can look from center
#define BLINK_CLOSE_Y 110  // How far eyelids close (120 = fully closed)

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
    Fusion_Init(0.0f, 0.0f, 0.0f); // init x y theta
    Init_All();

    // --- Initialisation du Wi-Fi ---
    if (cyw43_arch_init()) {
        printf("Échec de l'initialisation du Wi-Fi\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();

    printf("Connexion au Wi-Fi...\n");
    // Remplace par ton SSID et Mot de passe
    if (cyw43_arch_wifi_connect_timeout_ms("Opossum", "r28w3fr7j3zu8r4", CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Échec de la connexion Wi-Fi\n");
        return 1;
    } else {
        printf("Wi-Fi connecté !\n");
        // Optionnel : afficher l'adresse IP pour savoir où se connecter
        extern cyw43_t cyw43_state;
        uint32_t ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
        printf("Adresse IP: %d.%d.%d.%d\n", 
            ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
    }

    // --- Démarrage du serveur TCP ---
    tcp_server_t *tcp_state = tcp_server_open();
    if (!tcp_state) {
        printf("Erreur lors de l'ouverture du serveur TCP\n");
        // On peut continuer sans le Wi-Fi, mais on prévient
    } else {
        printf("Serveur TCP prêt sur le port %d\n", TCP_SERVER_PORT);
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

        // --- A. Screen Update ---
        // On met à jour l'animation à chaque tour de boucle
        // minion_eye_update_non_blocking();
        // --- B. Robot Logic ---
        bool has_data = false;
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
                if(LD19.newScan){
                    LD19.newScan = 0;
                    has_data = true;
                }
                if(has_data){
                    // 1. On récupère la position actuelle de la fusion (EN MÈTRES)
                    // RobotPose current_belief = Fusion_GetState();
                    
                    // 2. On la convertit EN MILLIMÈTRES pour aider la localisation
                    // // (au cas où Loc_ProcessScan s'en sert pour filtrer ses données)
                    // RobotPose belief_for_loc = current_belief;
                    // belief_for_loc.x *= 1000.0f;
                    // belief_for_loc.y *= 1000.0f;

                    // // 3. La localisation fait son calcul et sort un résultat (EN MILLIMÈTRES)
                    // RobotPose measured = Loc_ProcessScan(LD19.previousScan, &belief_for_loc);
                    
                    // if (measured.valid){
                    //     // 4. On convertit la mesure validée EN MÈTRES avant de l'envoyer à la fusion
                    //     measured.x /= 1000.0f;
                    //     measured.y /= 1000.0f;
                        
                    //     Fusion_Correct(measured);
                    // }
                    // LD19_printScanTeleplot(&LD19);
                }

                // Pour l'affichage, on reconvertit en mm si nécessaire
                // RobotPose final = Fusion_GetState();
                // if(Timer_ms1 % 100 == 0){
                //     // final.x et final.y sont en mètres, on les multiplie par 1000 pour l'affichage (si ton interface attend des mm)
                //     printf(">robot:%d:%d|xy,clr\n", (int)(final.x * 1000.0f), (int)(final.y * 1000.0f));
                //     printf(">room:0:0;1000:0;1000:2000;0:2000;0:0|xy,clr\n");
                // }
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
                // Ici tu peux ajouter d'autres tâches à faire régulièrement
                // Par exemple, vérifier des capteurs, envoyer des données au serveur TCP, etc.
                sequencer++;
                cyw43_arch_poll();
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
                timer_match = Timer_ms1;
                match_state++;
            }
            break;
        case 1:
            if ((Timer_ms1 - timer_match) > timer_match_delay && (Timer_ms1 - timer_match) < timer_match_delay_endgame) {
                // Actions du début de match (ex: se déplacer à un endroit stratégique)
                Goal_Pos.x = 0.6;
                Goal_Pos.y = 0.0;
                Goal_Pos.t = 0.0;
                motion_pos(Goal_Pos);
                timer_match = Timer_ms1; // reset timer for next phase
                match_state++;
            }
            break;
        case 2:
                if ((Timer_ms1 - timer_match) > 5000) {
                    timer_match = Timer_ms1; // reset timer for next phase
                    match_state++;
                }
                break;
        case 3:
            if (motion_done == 1) {
                Goal_Pos.x = 0.6;
                Goal_Pos.y = 0.0;
                Goal_Pos.t = 3.14159; // 180 degrees in radians
                motion_pos(Goal_Pos);
                match_state++;
            }
            break;
        case 4:
            if ((Timer_ms1 - timer_match) > 5000) {
                timer_match = Timer_ms1; // reset timer for next phase
                match_state++;
            }
            break;
        case 5:
            if (motion_done == 1) {
                Goal_Pos.x = 0.0;
                Goal_Pos.y = 0.0;
                Goal_Pos.t = 3.14159; // 180 degrees in radians
                motion_pos(Goal_Pos);
                match_state++;
            }
            break;
        default:
            break;
    }
}