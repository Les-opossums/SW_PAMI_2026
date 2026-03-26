#include "../PAMI_2026.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_TWO_PI
#define M_TWO_PI 6.28318530717958647692f
#endif

// -- internal constants --
#define HIST_RES 20.0f // 20 mm per bin
#define HIST_SIZE 300 // range coverage
#define HIST_CENTER (HIST_SIZE / 2) // center index of histogram
#define MIN_WALL_PTS 5 // minimum number of points to consider a wall valid

// Helpers mathématiques
static float normalize_angle(float angle) {
    while (angle < 0.0f) angle += M_TWO_PI;
    while (angle >= M_TWO_PI) angle -= M_TWO_PI;
    return angle;
}

static float angle_diff_rad(float target, float current) {
    float diff = target - current;
    while (diff < -M_PI) diff += M_TWO_PI;
    while (diff > M_PI)  diff -= M_TWO_PI;
    return diff;
}

static float find_grid_alignement_rad(const LD19DataPointHandler *scan) {
    int bins[90] = {0}; // histogram for angles for 0 to 90 degrees
    int step = 4; // step between points to consider to get a stable vector

    float rad_to_bin = 90.0f / (M_PI / 2.0f); // 90 bins for 90 degrees

    for (int i = 0; i < scan->index - step; i++){
        // ignore invalid or out of range points
        if(scan->points[i].distance < 100 || scan->points[i].distance > 3000) continue;

        // compute vector between points
        float dx = scan->points[i + step].x - scan->points[i].x;
        float dy = scan->points[i + step].y - scan->points[i].y;
        float d2 = dx*dx + dy*dy;
 
        // if distance squared is less than 10m, consider it for histogram (same object)
        if(d2 < 10000.0f){
            // atan2 gives direction of the wall
            float angle = atan2f(dy, dx);
            float norm = normalize_angle(angle);

            // map the angle to a bin index
            int bin_idx = (int)(norm * rad_to_bin) % 90;
            bins[bin_idx]++;
        }
    }

    // find the bin with the maximum count
    int max_val = 0;
    int max_idx = 0;
    // start loop at 1 to allow smoothing (checking neighbors)
    for (int i = 1; i < 89; i++) {
        int val = bins[i - 1] + bins[i]*2 + bins[i + 1];
        if (val > max_val) {
            max_val = val;
            max_idx = i;
        }
    }
    // convert the winning bin index back to radians
    float best_angle = ((float)max_idx) / rad_to_bin;
    return best_angle;
}

RobotPose Loc_ProcessScan(const LD19DataPointHandler* scan, RobotPose* prev_pose) {
    RobotPose result = {0};
    result.valid = false;

    // safety check: don't run if not enough points
    if (scan->index < 50) {
        return result;
    }

    // step 1 : determine heading modulo 90 degrees
    float align_angle_mod90 = find_grid_alignement_rad(scan);
    
    // --- LEVÉE D'AMBIGUÏTÉ (Unwrapping) ---
    // On cherche le multiple de 90° qui correspond le mieux au cap estimé du robot
    float best_diff = 1000.0f;
    float global_theta = align_angle_mod90;
    
    // On teste les 4 orientations possibles de la table
    for (int i = -2; i <= 2; i++) {
        float test_theta = align_angle_mod90 + (i * M_PI / 2.0f);
        float diff = fabsf(angle_diff_rad(prev_pose->theta, test_theta));
        
        if (diff < best_diff) {
            best_diff = diff;
            global_theta = normalize_angle(test_theta);
        }
    }

    // Si le Lidar propose un cap aberrant par rapport à l'odométrie (ex: robot emporté), on rejette
    if (best_diff > (M_PI / 4.0f)) { 
        return result;
    }

    // step 2 : rotate points to align with global frame
    float cos_theta = cosf(global_theta);
    float sin_theta = sinf(global_theta);

    // step 3 : build histograms for X and Y positions
    uint16_t x_hist[HIST_SIZE] = {0};
    uint16_t y_hist[HIST_SIZE] = {0};

    for (int i = 0; i < scan->index; i++) {
        // ignore invalid or out of range points
        if(scan->points[i].distance < 50 || scan->points[i].distance > 3500) continue;

        // ---------------------------------------------------------
        // CORRECTION DES AXES : On force le Lidar à correspondre à l'odométrie.
        // L'avant du Lidar (axe Y) devient l'avant du robot (axe X).
        // ---------------------------------------------------------
        // 1. FORCER L'ALIGNEMENT DU CAPTEUR AVEC LE ROBOT
        float lidar_x = scan->points[i].y;
        float lidar_y = -scan->points[i].x;

        // 2. ROTATION AVEC LES BONNES VARIABLES
        float x_rot = lidar_x * cos_theta - lidar_y * sin_theta;
        float y_rot = lidar_x * sin_theta + lidar_y * cos_theta;

        // compute histogram indices
        int x_idx = (int)(x_rot / HIST_RES) + HIST_CENTER;
        int y_idx = (int)(y_rot / HIST_RES) + HIST_CENTER;

        // accumulate in histograms if within bounds
        if (x_idx >= 0 && x_idx < HIST_SIZE) {
            x_hist[x_idx]++;
        }
        if (y_idx >= 0 && y_idx < HIST_SIZE) {
            y_hist[y_idx]++;
        }
    }

    // step 4: find wall boundaries (outside-in search)
    int left_idx = -1; 
    int right_idx = -1;
    int top_idx = -1;
    int bottom_idx = -1;

    // find left wall
    for (int i = 0; i < HIST_CENTER; i++) {
        if (x_hist[i] >= MIN_WALL_PTS) {
            left_idx = i;
            break;
        }
    }
    // find right wall
    for (int i = HIST_SIZE - 1; i >= HIST_CENTER; i--) {
        if (x_hist[i] >= MIN_WALL_PTS) {
            right_idx = i;
            break;
        }
    }
    // find bottom wall
    for (int i = 0; i < HIST_CENTER; i++) {
        if (y_hist[i] >= MIN_WALL_PTS) {
            bottom_idx = i;
            break;
        }
    }
    // find top wall
    for (int i = HIST_SIZE - 1; i >= HIST_CENTER; i--) {
        if (y_hist[i] >= MIN_WALL_PTS) {
            top_idx = i;
            break;
        }
    }

    // if any wall is missing, return invalid
    if (left_idx == -1 || right_idx == -1 || top_idx == -1 || bottom_idx == -1) {
        return result;
    }

    // step 5: calculate measured room dimensions
    float span_x = (right_idx - left_idx) * HIST_RES;
    float span_y = (top_idx - bottom_idx) * HIST_RES;

    // calculate distance from robot to the left and bottom walls
    float dist_to_left = (left_idx - HIST_CENTER) * HIST_RES;
    float dist_to_bottom = (bottom_idx - HIST_CENTER) * HIST_RES;

    // step 6: compute robot position in global frame
    if (fabsf(span_x - TABLE_SIZE_X) < LOC_TOLERANCE_MM &&
        fabsf(span_y - TABLE_SIZE_Y) < LOC_TOLERANCE_MM) {
        
        float calculated_x = -dist_to_left;
        float calculated_y = -dist_to_bottom;
        
        // --- FILTRAGE PAR DISTANCE (GATING) ---
        // On calcule le saut de position par rapport à l'estimation précédente
        float dx = calculated_x - prev_pose->x;
        float dy = calculated_y - prev_pose->y;
        float dist_jump = sqrtf(dx*dx + dy*dy);
        
        // Seuil de 300mm max. Si un robot masque le Lidar, la position trouvée 
        // sera très éloignée de la réalité et on la rejette.
        if (dist_jump <= 300.0f) {
            result.x = calculated_x;
            result.y = calculated_y;
            result.theta = global_theta;
            result.valid = true;
        }
        
        return result;
    }
    // CASE B : rotated 90 degrees (on rejette explicitement pour éviter l'inversion X/Y)
    else if (fabsf(span_x - TABLE_SIZE_Y) < LOC_TOLERANCE_MM &&
             fabsf(span_y - TABLE_SIZE_X) < LOC_TOLERANCE_MM) {
        return result; 
    }

    // Sécurité : toujours retourner result (qui est invalid par défaut ici)
    return result;
}