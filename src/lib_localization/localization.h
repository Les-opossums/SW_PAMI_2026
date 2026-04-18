#ifndef LOCALIZATION_H
#define LOCALIZATION_H

// ==========================================
// --- SÉLECTEUR DE MODE DE JEU ---
// ==========================================
// #define HOME_TEST_MODE 1 

#if HOME_TEST_MODE == 1
    // --- Configuration Petit Plateau (1/3 côté Jaune) ---
    #define TABLE_SIZE_X 1000.0f  // La table est coupée à 1 mètre
    #define TABLE_SIZE_Y 2000.0f  // La largeur reste la même
    
    // désactive la détection pour que le Lidar cherche le vrai mur de ta table à X=1000
    #define SCENE_DETECTION_ENABLED 0 
#else
    // --- Configuration Officielle Coupe 2026 ---
    #define TABLE_SIZE_X 3000.0f  
    #define TABLE_SIZE_Y 2000.0f
    #define SCENE_DETECTION_ENABLED 1
#endif

// --- Paramètres de la Scène 2026 (Fixes) ---
#define SCENE_X_MIN 600.0f
#define SCENE_X_MAX 2400.0f
#define SCENE_Y_MIN 1550.0f
#define SCENE_Y_MAX 2000.0f
#define SCENE_MARGIN_MM 150.0f 

// --- Configuration Histogrammes ---
#define HIST_RES 20.0f 
#define HIST_SIZE 300 
#define HIST_CENTER (HIST_SIZE / 2)
#define MIN_WALL_PTS 5
#define SEARCH_WINDOW_MM 400.0f
#define LOC_TOLERANCE_MM 1000.0f

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