#ifndef LOCALIZATION_H
#define LOCALIZATION_H

// table configuration
#define TABLE_SIZE_X 3000.0f  // in mm
#define TABLE_SIZE_Y 2000.0f  // in mm

// Si 0, le robot ignore la scène et cherche les bords extérieurs
#define SCENE_DETECTION_ENABLED 1

// Coordonnées du rectangle de la scène
#define SCENE_X_MIN 600.0f
#define SCENE_X_MAX 2400.0f
#define SCENE_Y_MIN 1550.0f
#define SCENE_Y_MAX 2000.0f

// Marge de sécurité pour passer en mode "détection de bord de scène"
// Évite les sauts de localisation quand on frôle le coin de la scène
#define SCENE_MARGIN_MM 150.0f 

// --- Configuration Histogrammes ---
#define HIST_RES 20.0f 
#define HIST_SIZE 300 
#define HIST_CENTER (HIST_SIZE / 2)
#define MIN_WALL_PTS 5
#define SEARCH_WINDOW_MM 400.0f

#define LOC_TOLERANCE_MM 1000.0f  // in mm

#define LD19_MAX_PTS_SCAN 1200

extern float table_size_x;
extern float table_size_y;

typedef struct {
    float x;      // in mm
    float y;      // in mm
    float theta;  // in radians
    bool valid;
} RobotPose;    


RobotPose Loc_ProcessScan(const LD19DataPointHandler* scan, RobotPose* prev_pose);

void set_table_size(float size_x, float size_y);
#endif // LOCALIZATION_H