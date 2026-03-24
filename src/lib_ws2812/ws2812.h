#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>

// Ton PIN pour la LED
#define LED_PIN 12

// Initialise le PIO pour la LED
void led_rgb_init(void);

// Modifie la couleur de la LED (valeurs de 0 à 255)
void led_rgb_set_color(uint8_t r, uint8_t g, uint8_t b);
uint8_t led_cmd(void);

#endif // WS2812_H