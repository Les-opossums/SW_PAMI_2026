#include "../PAMI_2026.h"
// Ce fichier sera généré automatiquement par CMake à partir de ws2812.pio
#include "ws2812.pio.h" 

#define IS_RGBW false  // La WS2812B classique est RGB, pas RGBW
#define WS2812_FREQ 800000 // 800 kHz, la fréquence standard du WS2812B

static PIO pio = pio0; // On utilise le bloc PIO 0 du Pico
static int sm = 0;     // State Machine 0

void led_rgb_init(void) {
    // Charge le programme PIO dans la mémoire du Pico
    uint offset = pio_add_program(pio, &ws2812_program);
    
    // Initialise le programme (cette fonction est définie à la fin de ws2812.pio)
    ws2812_program_init(pio, sm, offset, LED_PIN, WS2812_FREQ, IS_RGBW);
}

// Convertit le RGB en format GRB attendu par le WS2812B
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

void led_rgb_set_color(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t color = urgb_u32(r, g, b);
    // On décale de 8 bits vers la gauche car le PIO est configuré pour envoyer 24 bits
    pio_sm_put_blocking(pio, sm, color << 8u);
}

uint8_t led_cmd(void) {
    uint32_t r, g, b;
    if (Get_Param_u32(&r) || Get_Param_u32(&g) || Get_Param_u32(&b)) {
        return 1; // Erreur de paramètre
    }
    if (r > 255 || g > 255 || b > 255) {
        return 1; // Valeurs hors de portée
    }
    led_rgb_set_color((uint8_t)r, (uint8_t)g, (uint8_t)b);
    return 0; // Succès
}