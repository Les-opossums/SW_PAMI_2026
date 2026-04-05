/**
 * @file startup_screen.c
 * @brief Écran de démarrage PAMI 2026 — Afficheur rond 240×240
 *
 * Affiche :
 *   - L'ID du robot en grands chiffres 7 segments, colorés selon l'équipe
 *   - Un halo coloré derrière l'ID
 *   - La tension de batterie avec une jauge graphique
 *   - La position X, Y, θ mesurée par le Lidar
 *
 * Appel dans main() :
 *   startup_screen_show(&tft, current_config.pami_id, vbat,
 *                        pos.x * 1000.0f, pos.y * 1000.0f, pos.theta,
 *                        team_state);
 */

#include "../PAMI_2026.h"

// ─── Palette RGB565 ───────────────────────────────────────────────────────────
#define SC_BLACK     0x0000u
#define SC_WHITE     0xFFFFu
#define SC_LGRAY     0xCE59u   // Gris clair (labels)
#define SC_GRAY      0x4208u   // Gris sombre (séparateurs)
#define SC_BLUE      0x001Fu   // Bleu équipe
#define SC_YELLOW    0xFFE0u   // Jaune équipe
#define SC_DKBLUE    0x000Du   // Bleu très sombre (halo)
#define SC_DKYELLOW  0x5280u   // Ambre sombre (halo)
#define SC_GREEN     0x07E0u
#define SC_LGREEN    0x3FE6u
#define SC_ORANGE    0xFD20u
#define SC_RED       0xF800u

// ─── Police 5×7 (column-major, bit 0 = ligne du haut, ASCII 32–126) ───────────
// Source : Adafruit GFX glcdfont.c (domaine public)
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 32 ' '
    {0x00,0x00,0x5F,0x00,0x00}, // 33 '!'
    {0x00,0x07,0x00,0x07,0x00}, // 34 '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // 35 '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // 36 '$'
    {0x23,0x13,0x08,0x64,0x62}, // 37 '%'
    {0x36,0x49,0x55,0x22,0x50}, // 38 '&'
    {0x00,0x05,0x03,0x00,0x00}, // 39 '''
    {0x00,0x1C,0x22,0x41,0x00}, // 40 '('
    {0x00,0x41,0x22,0x1C,0x00}, // 41 ')'
    {0x14,0x08,0x3E,0x08,0x14}, // 42 '*'
    {0x08,0x08,0x3E,0x08,0x08}, // 43 '+'
    {0x00,0x50,0x30,0x00,0x00}, // 44 ','
    {0x08,0x08,0x08,0x08,0x08}, // 45 '-'
    {0x00,0x60,0x60,0x00,0x00}, // 46 '.'
    {0x20,0x10,0x08,0x04,0x02}, // 47 '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // 48 '0'
    {0x00,0x42,0x7F,0x40,0x00}, // 49 '1'
    {0x42,0x61,0x51,0x49,0x46}, // 50 '2'
    {0x21,0x41,0x45,0x4B,0x31}, // 51 '3'
    {0x18,0x14,0x12,0x7F,0x10}, // 52 '4'
    {0x27,0x45,0x45,0x45,0x39}, // 53 '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // 54 '6'
    {0x41,0x21,0x11,0x09,0x07}, // 55 '7'
    {0x36,0x49,0x49,0x49,0x36}, // 56 '8'
    {0x46,0x49,0x49,0x29,0x1E}, // 57 '9'
    {0x00,0x36,0x36,0x00,0x00}, // 58 ':'
    {0x00,0x56,0x36,0x00,0x00}, // 59 ';'
    {0x08,0x14,0x22,0x41,0x00}, // 60 '<'
    {0x14,0x14,0x14,0x14,0x14}, // 61 '='
    {0x00,0x41,0x22,0x14,0x08}, // 62 '>'
    {0x02,0x01,0x51,0x09,0x06}, // 63 '?'
    {0x32,0x49,0x79,0x41,0x3E}, // 64 '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 65 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 66 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 67 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 68 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 69 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 70 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 71 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 72 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 73 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 74 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 75 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 76 'L'
    {0x7F,0x02,0x04,0x02,0x7F}, // 77 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 78 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 79 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 80 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 81 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 82 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 83 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 84 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 85 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 86 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 87 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 88 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 89 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 90 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // 91 '['
    {0x02,0x04,0x08,0x10,0x20}, // 92 '\'
    {0x00,0x41,0x41,0x7F,0x00}, // 93 ']'
    {0x04,0x02,0x01,0x02,0x04}, // 94 '^'
    {0x40,0x40,0x40,0x40,0x40}, // 95 '_'
    {0x00,0x01,0x02,0x04,0x00}, // 96 '`'
    {0x20,0x54,0x54,0x54,0x78}, // 97 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 98 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 99 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 100 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 101 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 102 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 103 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 104 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 105 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 106 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 107 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 108 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 109 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 110 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 111 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 112 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 113 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 114 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 115 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 116 't'
    {0x3C,0x40,0x40,0x40,0x7C}, // 117 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 118 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 119 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 120 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 121 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 122 'z'
    {0x00,0x08,0x36,0x41,0x00}, // 123 '{'
    {0x00,0x00,0x77,0x00,0x00}, // 124 '|'
    {0x00,0x41,0x36,0x08,0x00}, // 125 '}'
    {0x08,0x04,0x08,0x10,0x08}, // 126 '~'
};

// ─── Données afficheur 7 segments ─────────────────────────────────────────────
// Bits : A=top(0x01) B=haut-D(0x02) C=bas-D(0x04) D=bas(0x08)
//        E=bas-G(0x10) F=haut-G(0x20) G=milieu(0x40)
static const uint8_t seg7[10] = {
    0x3F, // 0 : A B C D E F
    0x06, // 1 : B C
    0x5B, // 2 : A B D E G
    0x4F, // 3 : A B C D G
    0x66, // 4 : B C F G
    0x6D, // 5 : A C D F G
    0x7D, // 6 : A C D E F G
    0x07, // 7 : A B C
    0x7F, // 8 : A B C D E F G
    0x6F, // 9 : A B C D F G
};

// ─── Primitives de dessin ─────────────────────────────────────────────────────

static void sc_fill_rect(uint16_t *buf, int x, int y, int w, int h, uint16_t color)
{
    uint16_t sw = __builtin_bswap16(color);
    int y2 = y + h, x2 = x + w;
    for (int row = (y < 0 ? 0 : y); row < y2 && row < 240; row++) {
        for (int col = (x < 0 ? 0 : x); col < x2 && col < 240; col++) {
            buf[row * 240 + col] = sw;
        }
    }
}

// Ligne horizontale pleine (très rapide via memset-style)
static void sc_hline(uint16_t *buf, int x, int y, int w, uint16_t color)
{
    if (y < 0 || y >= 240) return;
    uint16_t sw = __builtin_bswap16(color);
    int x2 = x + w;
    for (int col = (x < 0 ? 0 : x); col < x2 && col < 240; col++)
        buf[y * 240 + col] = sw;
}

// Dessine un caractère (scale 1 = 5×7, scale 2 = 10×14, ...)
static void sc_draw_char(uint16_t *buf, int x, int y, char c, uint16_t color, int scale)
{
    if (c < 32 || c > 126) c = '?';
    const uint8_t *g = font5x7[(uint8_t)c - 32];
    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 7; row++) {
            if (g[col] & (1 << row)) {
                sc_fill_rect(buf, x + col * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}

// Largeur en pixels d'une chaîne (sans espace de fin)
static int sc_str_width(const char *str, int scale)
{
    int n = strlen(str);
    if (n == 0) return 0;
    return n * (5 * scale + scale) - scale; // 6*scale par char, -1 trailing gap
}

// Chaîne centrée sur cx
static void sc_draw_str_centered(uint16_t *buf, int cx, int y,
                                  const char *str, uint16_t color, int scale)
{
    int x = cx - sc_str_width(str, scale) / 2;
    for (; *str; str++, x += 6 * scale)
        sc_draw_char(buf, x, y, *str, color, scale);
}

// ─── Afficheur 7 segments ─────────────────────────────────────────────────────
// (x0, y0) = coin supérieur-gauche du chiffre
// W = largeur totale, H = hauteur totale, T = épaisseur des barres
static void sc_draw_seg7(uint16_t *buf, int x0, int y0,
                          int digit, uint16_t color,
                          int W, int H, int T)
{
    if (digit < 0 || digit > 9) return;
    uint8_t s = seg7[digit];
    int hh = H / 2; // demi-hauteur

    // A — barre du haut
    if (s & 0x01) sc_fill_rect(buf, x0 + T,     y0,              W - 2*T, T,          color);
    // B — barre haut-droite
    if (s & 0x02) sc_fill_rect(buf, x0 + W - T, y0 + T,          T,       hh - 2*T,  color);
    // C — barre bas-droite
    if (s & 0x04) sc_fill_rect(buf, x0 + W - T, y0 + hh + T,     T,       hh - 2*T,  color);
    // D — barre du bas
    if (s & 0x08) sc_fill_rect(buf, x0 + T,     y0 + H - T,      W - 2*T, T,          color);
    // E — barre bas-gauche
    if (s & 0x10) sc_fill_rect(buf, x0,         y0 + hh + T,     T,       hh - 2*T,  color);
    // F — barre haut-gauche
    if (s & 0x20) sc_fill_rect(buf, x0,         y0 + T,          T,       hh - 2*T,  color);
    // G — barre du milieu
    if (s & 0x40) sc_fill_rect(buf, x0 + T,     y0 + hh - T/2,   W - 2*T, T,          color);
}

// Dessine l'ID en 7 segments (1, 2 ou 3 chiffres), centré sur cx
static void sc_draw_id(uint16_t *buf, int id, uint16_t color)
{
    if (id < 10) {
        // ── 1 chiffre : très grand ──
        int W=68, H=90, T=9;
        sc_draw_seg7(buf, 120 - W/2, 40, id, color, W, H, T);
    } else if (id < 100) {
        // ── 2 chiffres ──
        int W=52, H=80, T=8, G=12;
        int total = 2*W + G;
        int x0 = 120 - total/2;
        sc_draw_seg7(buf, x0,         44, id / 10,  color, W, H, T);
        sc_draw_seg7(buf, x0 + W + G, 44, id % 10,  color, W, H, T);
    } else {
        // ── 3 chiffres : plus compact ──
        int W=38, H=66, T=7, G=8;
        int total = 3*W + 2*G;
        int x0 = 120 - total/2;
        sc_draw_seg7(buf, x0,           48, id / 100,       color, W, H, T);
        sc_draw_seg7(buf, x0 + W + G,   48, (id / 10) % 10, color, W, H, T);
        sc_draw_seg7(buf, x0 + 2*(W+G), 48, id % 10,        color, W, H, T);
    }
}

static void sc_draw_wifi(uint16_t *buf, int cx, int cy, bool connected)
{
    uint16_t col = connected ? SC_GREEN : SC_GRAY;
    uint16_t sw = __builtin_bswap16(col);

    for (int i = 0; i < 3; i++) {
        int r = 4 + i * 5;   // 🔽 plus petit (avant 6 + i*6)
        int thickness = 2;

        for (int y = -r; y <= 0; y++) {
            for (int x = -r; x <= r; x++) {
                int d2 = x*x + y*y;

                if (d2 <= r*r && d2 >= (r - thickness)*(r - thickness)) {
                    int px = cx + x;
                    int py = cy + y;

                    if (px >= 0 && px < 240 && py >= 0 && py < 240) {
                        buf[py * 240 + px] = sw;
                    }
                }
            }
        }
    }

    sc_fill_rect(buf, cx - 1, cy + 1, 3, 3, col); // 🔽 point plus petit
}

// ─── Jauge batterie ───────────────────────────────────────────────────────────
// 2S LiPo : 6.0 V (vide) → 8.4 V (pleine)
// Dessinée centrée sur cx
static void sc_draw_battery(uint16_t *buf, float voltage, int cx, int y)
{
    float pct = (voltage - 6.0f) / (8.4f - 6.0f);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 1.0f) pct = 1.0f;

    uint16_t bar_col;
    if      (pct > 0.60f) bar_col = SC_GREEN;
    else if (pct > 0.25f) bar_col = SC_ORANGE;
    else                  bar_col = SC_RED;

    // ── Corps de la jauge ──
    const int BW = 60, BH = 20; // Largeur et hauteur du corps
    const int NW = 5,  NH = 6;  // Embout positif (nub)
    const int PAD = 2;           // Padding intérieur

    int bx = cx - BW/2;         // x gauche du corps
    int by = y;

    // Contour (gris clair)
    sc_fill_rect(buf, bx,         by,       BW,  BH,  SC_GRAY);
    // Fond intérieur (noir)
    sc_fill_rect(buf, bx + 1,     by + 1,   BW-2, BH-2, SC_BLACK);
    // Embout (nub)
    int nx = bx + BW;
    int ny = by + (BH - NH) / 2;
    sc_fill_rect(buf, nx,         ny,       NW,   NH,  SC_GRAY);
    // Remplissage barre
    int fill_w = (int)((BW - 2*PAD) * pct);
    if (fill_w > 0)
        sc_fill_rect(buf, bx + PAD, by + PAD, fill_w, BH - 2*PAD, bar_col);

    // ── Pourcentage + tension ──
    char txt[16];
    int pct_int = (int)(pct * 100.0f + 0.5f);
    snprintf(txt, sizeof(txt), "%3d%%  %.1fV", pct_int, (double)voltage);
    sc_draw_str_centered(buf, cx, by + BH + 5, txt, SC_LGRAY, 1);
}

// ─── Fonction principale ──────────────────────────────────────────────────────

/**
 * @brief Affiche l'écran de démarrage pendant @p duration_ms millisecondes.
 *
 * @param tft           Pointeur vers le driver GC9A01A (déjà initialisé)
 * @param robot_id      ID du robot (0–255), affiché en grand
 * @param bat_voltage   Tension batterie en volts (ex: 7.6f)
 * @param pos_x_m     Position X Lidar en mètres (ex: 0.12f pour 12 cm, -0.34f pour -34 cm)
 * @param pos_y_m     Position Y Lidar en mètres (ex: 0.12f pour 12 cm, -0.34f pour -34 cm)
 * @param pos_theta_rad Angle θ en radians
 * @param team_color    0 = BLEU, 1 = JAUNE
 * @param emergency_stop true si arrêt d’urgence actif (affiche une alerte rouge)
 * @param wifi_connected true si WiFi connecté (affiche l'icône en vert
 * @param duration_ms   Durée d'affichage (3000 ms recommandé)
 */
void startup_screen_show(gc9a01a_t *tft,
                          uint8_t  robot_id,
                          float    bat_voltage,
                          float    pos_x_m,
                          float    pos_y_m,
                          float    pos_theta_rad,
                          uint8_t  team_color,
                          bool     emergency_stop,
                          bool     wifi_connected,
                          uint32_t duration_ms)
{
    while (gc9a01a_is_busy()) tight_loop_contents();

    uint16_t *buf = gc9a01a_draw_buffer;

    uint16_t col_team  = (team_color == 0) ? SC_BLUE   : SC_YELLOW;
    uint16_t col_dark  = (team_color == 0) ? SC_DKBLUE : SC_DKYELLOW;
    const char *team_str = (team_color == 0) ? "BLEU" : "JAUNE";

    gc9a01a_fill_screen(SC_BLACK);

    // HEADER
    sc_draw_str_centered(buf, 120, 10, "PAMI 2026", SC_GRAY, 1);

    // ═════ ZONE ID ═════
    sc_fill_rect(buf, 0, 30, 100, 120, col_dark);

    if (robot_id < 10) {
        sc_draw_seg7(buf, 20, 50, robot_id, col_team, 60, 90, 8);
    } else {
        sc_draw_seg7(buf, 5, 55, robot_id / 10, col_team, 40, 70, 6);
        sc_draw_seg7(buf, 50, 55, robot_id % 10, col_team, 40, 70, 6);
    }

    sc_draw_str_centered(buf, 50, 130, team_str, SC_WHITE, 1);

    // ═════ POSITION ═════
    char line_x[20], line_y[20], line_t[20];

    snprintf(line_x, sizeof(line_x), "X:%+.2f m", (double)pos_x_m);
    snprintf(line_y, sizeof(line_y), "Y:%+.2f m", (double)pos_y_m);

    float deg = pos_theta_rad * (180.0f / 3.14159265f);
    snprintf(line_t, sizeof(line_t), "T:%+.1f deg", (double)deg);

    sc_draw_str_centered(buf, 170, 40, "POSITION", col_team, 1);

    sc_draw_str_centered(buf, 170, 65, line_x, SC_WHITE, 2);
    sc_draw_str_centered(buf, 170, 95, line_y, SC_WHITE, 2);
    sc_draw_str_centered(buf, 170, 125, line_t, SC_WHITE, 2);

    // ═════ SEPARATEUR ═════
    sc_hline(buf, 10, 160, 220, SC_GRAY);

    // ═════ ZONE BATTERIE ═════

    // Fond alerte si arrêt d’urgence
    if (emergency_stop) {
        sc_fill_rect(buf, 0, 175, 240, 65, SC_RED);
    }

    // WiFi à droite
    sc_draw_wifi(buf, 60, 205, wifi_connected);

    // Batterie centrée un peu plus haut
    sc_draw_battery(buf, bat_voltage, 150, 185);

    // ═════ ENVOI ═════
    gc9a01a_update_async(tft);

    sleep_ms(duration_ms);
    while (gc9a01a_is_busy()) tight_loop_contents();
}