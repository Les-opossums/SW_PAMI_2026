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

    Path_Init();
    // Launch Lidar on Core 1
    multicore_launch_core1(core1_entry);

    printf("PAMI-2026 ready (Lidar + Screen).\n");

    while (true) {
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
                // Asserv_Loop();
                if (Timer_ms1 % freq_robot_data_update == 0 && Timer_ms1 != last_robot_data_update_time) { // e.g., 20 Hz
                    // printf("ROBOTDATA 1 2 3 4 5 6\n");

                    last_robot_data_update_time = Timer_ms1;
                }
                sequencer++;
                break;
            case 2:
                if(LD19.newScan){
                    LD19.newScan = 0;
                    has_data = true;
                }
                if(has_data){
                    // // 1. On récupère la position actuelle de la fusion (EN MÈTRES)
                    // RobotPose current_belief = Fusion_GetState();
                    
                    // // 2. On la convertit EN MILLIMÈTRES pour aider la localisation
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
                sequencer = 0;
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
}

uint8_t FREQ_Cmd(void) {
    uint32_t val32;
    if (Get_Param_u32(&val32)){
        return PARAM_ERROR_CODE;
    }
    freq_robot_data_update = (int)val32;
    return 0;
}