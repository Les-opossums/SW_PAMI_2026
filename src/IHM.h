#ifndef PAMI_2026_IHM_H
#define PAMI_2026_IHM_H

#define DEBOUNCE_DELAY_MS 50

typedef struct {
    int ID;
    int screen_mode; // 0 = startup, 1 = config, 2 = match
    bool wifi_connected;
    bool au_state; // 0 = AU mode on, 1 = AU mode off
    bool team_state; // 0 = BLUE, 1 = YELLOW
    bool leash_state;
    float current_vbat;

    bool match_started_once; // Latch pour afficher l'écran de démarrage uniquement au premier démarrage
    bool start_match; // État actuel du match (0 = non démarré, 1 = démarré)

    bool interaction_detected; // Flag pour détecter une interaction récente (changement d'état GPIO)
} IHM_Struct_t;

extern IHM_Struct_t IHM;

void IHM_init(void);

void IHM_get_AU_state(void);
void IHM_get_team_state(void);
void IHM_get_leash_state(void);

void IHM_get_battery_voltage(void);

#endif // PAMI_2026_IHM_H