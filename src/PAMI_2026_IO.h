#ifndef PAMI_2026_IO_H
#define PAMI_2026_IO_H

// IO for driver stepper motor PAMI-2026
#define PIN_STEP_1 7
#define PIN_DIR_1 8

#define PIN_STEP_2 17
#define PIN_DIR_2 16

#define PIN_STEP_3 2
#define PIN_DIR_3 3


#define LEASH_PIN 9

#define AU_PIN 6

#define TEAM_PIN 10

// ADC for battery voltage reading
#define ADC_VBAT_CHANNEL 0
#define ADC_VBAT_PIN 26


// Pin configuration
#define LCD_CS_PIN 21
#define LCD_DC_PIN 20
#define LCD_RST_PIN 22 // Use -1 if you skip the reset pin

#endif // PAMI_2026_IO_H