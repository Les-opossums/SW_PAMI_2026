#include "PAMI_2026.h"

IHM_Struct_t IHM;

void IHM_init(void) {
    IHM.ID = current_config.pami_id;

    IHM.screen_mode = 0; // startup
    IHM.wifi_connected = false;
    IHM.au_state = false;
    IHM.team_state = false;
    IHM.leash_state = false;
    IHM.current_vbat = 0.0f;

    IHM.match_started_once = false;
    IHM.start_match = false;

    IHM.interaction_detected = false;

    gpio_init(LEASH_PIN);
    gpio_set_dir(LEASH_PIN, GPIO_IN);

    gpio_init(AU_PIN);
    gpio_set_dir(AU_PIN, GPIO_IN);
    gpio_pull_up(AU_PIN); // Ajouté depuis ton code d'origine

    gpio_init(TEAM_PIN);
    gpio_set_dir(TEAM_PIN, GPIO_IN);

    adc_init();
    adc_gpio_init(ADC_VBAT_PIN);
}


int previous_au_state = -1;

// Variables de gestion de temps pour l'antirebond
uint32_t last_debounce_time_au = 0;
uint32_t last_debounce_time_team = 0;
uint32_t last_debounce_time_leash = 0;

// On garde l'état "lu" (instable) pour le comparer au dernier état "validé"
int last_raw_au = -1;
int last_raw_team = -1;
int last_raw_leash = -1;

void IHM_get_AU_state(void) {
    uint32_t current_time = Timer_ms1; // Utilise ton timer global
    int current_raw = gpio_get(AU_PIN);

    // Si le signal physique change (même un parasite)
    if (current_raw != last_raw_au) {
        last_debounce_time_au = current_time; // On reset le chrono
        last_raw_au = current_raw;
    }

    // Si le signal est stable depuis plus de DEBOUNCE_DELAY_MS
    if ((current_time - last_debounce_time_au) > DEBOUNCE_DELAY_MS) {
        // Si l'état validé est différent de l'état précédent en mémoire
        if (current_raw != previous_au_state) {
            previous_au_state = current_raw;
            IHM.au_state = current_raw;
            IHM.interaction_detected = true;

            if(!IHM.au_state) {
                printf("AU ACTIVATED\n");
            } else {
                printf("AU DEACTIVATED\n");
            }
        }
    }
}


int previous_team_state = -1;

void IHM_get_team_state(void) {
    uint32_t current_time = Timer_ms1;
    int current_raw = gpio_get(TEAM_PIN);

    if (current_raw != last_raw_team) {
        last_debounce_time_team = current_time;
        last_raw_team = current_raw;
    }

    if ((current_time - last_debounce_time_team) > DEBOUNCE_DELAY_MS) {
        if (current_raw != previous_team_state) {
            previous_team_state = current_raw;
            IHM.team_state = current_raw;
            IHM.interaction_detected = true;

            if(IHM.team_state) {
                printf("TEAM YELLOW\n");
            } else {
                printf("TEAM BLUE\n");
            }
        }
    }
}

int previous_leash_state = -1;

void IHM_get_leash_state(void) {
    uint32_t current_time = Timer_ms1;
    int current_raw = gpio_get(LEASH_PIN);

    if (current_raw != last_raw_leash) {
        last_debounce_time_leash = current_time;
        last_raw_leash = current_raw;
    }

    if ((current_time - last_debounce_time_leash) > DEBOUNCE_DELAY_MS) {
        if (current_raw != previous_leash_state) {
            previous_leash_state = current_raw;
            IHM.leash_state = current_raw;
            IHM.interaction_detected = true;

            if(!IHM.leash_state) {
                IHM.start_match = 1;    
                IHM.match_started_once = true;
                printf("LEASH ACTIVATED\n");
            } else {
                IHM.start_match = 0;
                IHM.match_started_once = false;
                printf("LEASH DEACTIVATED\n");
            }
        }
    }

}

void IHM_get_battery_voltage(void) {
    adc_select_input(0);
    float adc_val = (float)adc_read();
    IHM.current_vbat = (adc_val / 4095.0f) * 9.9f;
    // printf("Battery Voltage: %.2f V\n", IHM.current_vbat);
}