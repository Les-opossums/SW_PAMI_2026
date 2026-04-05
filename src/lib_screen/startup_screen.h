
void startup_screen_show(gc9a01a_t *tft,
                          uint8_t  robot_id,
                          float    bat_voltage,
                          float    pos_x_mm,
                          float    pos_y_mm,
                          float    pos_theta_rad,
                          uint8_t  team_color,
                          bool     emergency_stop,
                          bool     wifi_connected,
                          uint32_t duration_ms);