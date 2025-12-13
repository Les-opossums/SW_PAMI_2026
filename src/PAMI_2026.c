#include "PAMI_2026.h"

// ==========================================
// --- Configuration SCREEN (Feature/Screen) ---
// ==========================================
#define CENTER_X 120
#define CENTER_Y 120
#define MAX_PUPIL_DIST 35  // How far the eye can look from center
#define BLINK_CLOSE_Y 110  // How far eyelids close (120 = fully closed)

// Pin configuration
#define LCD_CS_PIN 17
#define LCD_DC_PIN 16
#define LCD_RST_PIN 15 // Use -1 if you skip the reset pin

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
    gc9a01a_t tft;
    gc9a01a_init_struct(&tft, LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN);
    gc9a01a_begin(&tft); // Uses default SPI_DEFAULT_FREQ (40MHz)
    minion_eye_init(&tft);

    // 3. Initialize Robot Logic (HEAD)
    int sequencer = 0;
    Fusion_Init(500.0f, 500.0f, 0.0f); // init x y theta
    // Init_All();
    
    // Launch Lidar on Core 1
    multicore_launch_core1(core1_entry);

    printf("PAMI-2026 ready (Lidar + Screen).\n");

    while (true) {
        // --- A. Screen Update ---
        // On met à jour l'animation à chaque tour de boucle
        minion_eye_update_non_blocking();
        // --- B. Robot Logic ---
        bool has_data = false;
        Timer_Update(); // Met à jour les timers
        int c;

        // Met à jour les moteurs pas à pas
        // Move_Loop();

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
                sequencer++;
                break;
            case 2:
                if(LD19.newScan){
                    LD19.newScan = 0;
                    has_data = true;
                }
                if(has_data){
                    RobotPose current_belief = Fusion_GetState();
                    RobotPose measured = Loc_ProcessScan(LD19.previousScan, &current_belief);
                    if (measured.valid){
                        Fusion_Correct(measured);
                    }
                    // LD19_printScanTeleplot(&LD19);
                }

                RobotPose final = Fusion_GetState();
                if(Timer_ms1 % 100 == 0){
                    printf(">robot:%d:%d|xy,clr\n", (int)final.x, (int)final.y);
                    printf(">room:0:0;1000:0;1000:2000;0:2000;0:0|xy,clr\n");
                }
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