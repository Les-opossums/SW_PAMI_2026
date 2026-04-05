#ifndef PAMI_CONFIG_H
#define PAMI_CONFIG_H

// Structure de configuration stockée en Flash
typedef struct {
    uint32_t magic_number; // Pour vérifier si la flash a été initialisée
    uint8_t pami_id;       // Ton ID de 1 à 6
    uint8_t team_color;    // Optionnel : Bleu/Jaune stocké aussi ?
    uint16_t reserved;     // Padding pour alignement
} PAMI_Global_Config;

extern PAMI_Global_Config current_config;

void Config_Load(void);
void Config_Save(uint8_t new_id);
uint8_t Config_Register_Cmd(void);
uint8_t Get_Config_Cmd(void);

#endif