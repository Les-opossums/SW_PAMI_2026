#include "PAMI_2026.h"

// On définit l'adresse en fin de Flash (ex: 2Mo de Flash - 4ko)
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define MAGIC_KEY 0xFACEBEEF

PAMI_Global_Config current_config;

void Config_Load(void) {
    const uint8_t* flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
    memcpy(&current_config, flash_target_contents, sizeof(PAMI_Global_Config));

    if (current_config.magic_number != MAGIC_KEY) {
        printf("[Config] Flash vide ou corrompue. ID par défaut: 0\n");
        current_config.pami_id = 0;
        current_config.magic_number = MAGIC_KEY;
    } else {
        printf("[Config] PAMI ID chargé : %d\n", current_config.pami_id);
    }
}

void Config_Save(uint8_t new_id) {
    current_config.pami_id = new_id;
    current_config.magic_number = MAGIC_KEY;

    uint8_t buffer[FLASH_PAGE_SIZE];
    memset(buffer, 0, FLASH_PAGE_SIZE);
    memcpy(buffer, &current_config, sizeof(PAMI_Global_Config));

    printf("[Config] Mise en pause du Core 1 et sauvegarde...\n");

    // 1. Bloque le Core 1 pour qu'il arrête de lire la Flash
    multicore_lockout_start_blocking(); 
    
    // 2. Coupe les interruptions du Core 0
    uint32_t ints = save_and_disable_interrupts();
    
    // 3. Opérations Flash sécurisées
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, buffer, FLASH_PAGE_SIZE);
    
    // 4. Rétablit les interruptions et libère le Core 1
    restore_interrupts(ints);
    multicore_lockout_end_blocking();

    printf("[Config] ID %d sauvegardé en Flash avec succès !\n", new_id);
}

uint8_t Config_Register_Cmd(void){
        // Cette fonction doit être appelée dans l'initialisation de ton interpréteur pour enregistrer la commande "CONFIG"
        // Exemple : Register_Command("CONFIG", Config_Register_Cmd);
        // Usage : CONFIG 3  -> Sauvegarde l'ID 3 en Flash
        uint32_t new_id;
        if (Get_Param_u32(&new_id)) {
            printf("Usage: CONFIG <ID (0-255)>\n");
            return 1; // Code d'erreur pour paramètre manquant ou invalide
        }
        if (new_id > 255) {
            printf("Error: ID must be between 0 and 255.\n");
            return 1; // Code d'erreur pour ID hors limite
        }
        Config_Save((uint8_t)new_id);
        return 0; // Succès
}

uint8_t Get_Config_Cmd(void){
        // Cette fonction doit être appelée dans l'initialisation de ton interpréteur pour enregistrer la commande "GET_CONFIG"
        // Exemple : Register_Command("GET_CONFIG", Get_Config_Cmd);
        // Usage : GET_CONFIG  -> Affiche l'ID actuellement sauvegardé en Flash
        printf("Current PAMI ID: %d\n", current_config.pami_id);
        return 0; // Succès
}
