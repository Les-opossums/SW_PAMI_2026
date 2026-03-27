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

static float refine_peak(const uint16_t* hist, int peak_idx) {
    if (peak_idx < 1 || peak_idx >= HIST_SIZE - 1) return (float)peak_idx;
    
    float sum = 0.0f;
    float count = 0.0f;
    
    for (int i = peak_idx - 1; i <= peak_idx + 1; i++) {
        sum += i * hist[i];
        count += hist[i];
    }
    
    if (count > 0.0f) {
        return sum / count;
    }
    return (float)peak_idx;
}
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

static float find_angular_error(const LD19DataPointHandler *scan, float expected_theta) {
    int bins[90] = {0}; // histogram for angles for 0 to 90 degrees
    float rad_to_bin = 90.0f / (M_PI / 2.0f); // 90 bins for 90 degrees

    for (int i = 0; i < scan->index; i++){
        // ignore invalid or out of range points
        if(scan->points[i].distance < 100 || scan->points[i].distance > 3000) continue;

        int j = i+1;
        float dx, dy, d2;
        bool found = false;

        while(j < scan->index) {
            dx = scan->points[j].x - scan->points[i].x;
            dy = -(scan->points[j].y - scan->points[i].y); // inversion Y
            d2 = dx*dx + dy*dy;

            if(d2 > 22500.0f){ 
                if(d2 < 90000.0f){ // if distance squared is between 15cm and 30cm, stop looking for neighbors (different object)
                    found = true;
                }
                break;
            }
            j++;
        }
 
        if(found){
            float angle_lidar = atan2f(dy, dx);
            float angle_global_mesure = angle_lidar + expected_theta;

            float norm = angle_global_mesure;
            while (norm < 0.0f) norm += (M_PI / 2.0f);
            while (norm >= (M_PI / 2.0f)) norm -= (M_PI / 2.0f);

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

    if(max_val < 5) return 0.0f; // if not enough points, return no correction

    float dominant_angle_mod = ((float)max_idx) / rad_to_bin; // convert bin index back to angle

    float angular_error = dominant_angle_mod;
    if (angular_error > M_PI / 4.0f) {
        angular_error -= M_PI / 2.0f; // adjust to range [-45°, +45°]
    }
    return angular_error; 
}

RobotPose Loc_ProcessScan(const LD19DataPointHandler* scan, RobotPose* prev_pose) {
    RobotPose result = {0};
    result.valid = false;

    // safety check: don't run if not enough points
    if (scan->index < 50) {
        return result;
    }

    // step 1 : find and correct angular error using dominant angle of scan points
    float angular_error = find_angular_error(scan, prev_pose->theta);

    float global_theta = normalize_angle(prev_pose->theta - angular_error);

    // step 2 : precompute cos and sin for rotation
    float cos_theta = cosf(global_theta);
    float sin_theta = sinf(global_theta);

    uint16_t x_hist[HIST_SIZE] = {0};
    uint16_t y_hist[HIST_SIZE] = {0};

    // step 3 : build histograms for X and Y positions
    for (int i = 0; i < scan->index; i++) {
        if(scan->points[i].distance < 50 || scan->points[i].distance > 3500) continue;

        float lidar_x = scan->points[i].x;
        float lidar_y = -scan->points[i].y; // inversion Y car le lidar à la tête en bas

        // Rotation pour aligner les murs avec la table
        float x_rot = lidar_x * cos_theta - lidar_y * sin_theta;
        float y_rot = lidar_x * sin_theta + lidar_y * cos_theta;

        int x_idx = (int)(x_rot / HIST_RES) + HIST_CENTER;
        int y_idx = (int)(y_rot / HIST_RES) + HIST_CENTER;

        if (x_idx >= 0 && x_idx < HIST_SIZE) x_hist[x_idx]++;
        if (y_idx >= 0 && y_idx < HIST_SIZE) y_hist[y_idx]++;
    }

    // step 4: Windowed Peak Search (Recherche ciblée)
    // On cherche les murs uniquement autour de là où l'odométrie pense qu'ils sont (± 40 cm)
    int search_window = 400 / HIST_RES; 

    int expected_left_idx   = (int)(-prev_pose->x / HIST_RES) + HIST_CENTER;
    int expected_right_idx  = (int)((TABLE_SIZE_X - prev_pose->x) / HIST_RES) + HIST_CENTER;
    int expected_bottom_idx = (int)(-prev_pose->y / HIST_RES) + HIST_CENTER;
    int expected_top_idx    = (int)((TABLE_SIZE_Y - prev_pose->y) / HIST_RES) + HIST_CENTER;

    int left_idx = -1, right_idx = -1, bottom_idx = -1, top_idx = -1;
    int max_val;

    // Mur Gauche (X=0)
    max_val = 0;
    for (int i = expected_left_idx - search_window; i <= expected_left_idx + search_window; i++) {
        if (i >= 0 && i < HIST_SIZE && x_hist[i] > max_val && x_hist[i] >= MIN_WALL_PTS) {
            max_val = x_hist[i]; left_idx = i;
        }
    }
    // Mur Droit (X=2000)
    max_val = 0;
    for (int i = expected_right_idx - search_window; i <= expected_right_idx + search_window; i++) {
        if (i >= 0 && i < HIST_SIZE && x_hist[i] > max_val && x_hist[i] >= MIN_WALL_PTS) {
            max_val = x_hist[i]; right_idx = i;
        }
    }
    // Mur Bas (Y=0)
    max_val = 0;
    for (int i = expected_bottom_idx - search_window; i <= expected_bottom_idx + search_window; i++) {
        if (i >= 0 && i < HIST_SIZE && y_hist[i] > max_val && y_hist[i] >= MIN_WALL_PTS) {
            max_val = y_hist[i]; bottom_idx = i;
        }
    }
    // Mur Haut (Y=3000)
    max_val = 0;
    for (int i = expected_top_idx - search_window; i <= expected_top_idx + search_window; i++) {
        if (i >= 0 && i < HIST_SIZE && y_hist[i] > max_val && y_hist[i] >= MIN_WALL_PTS) {
            max_val = y_hist[i]; top_idx = i;
        }
    }

    // step 5: Compute robot position in global frame avec le Barycentre
    float calculated_x = prev_pose->x; 
    bool x_updated = false;
    
    // Si on voit l'un des deux murs X, on met à jour avec précision décimale
    if (left_idx != -1) {
        float exact_idx = refine_peak(x_hist, left_idx);
        calculated_x = -(exact_idx - HIST_CENTER) * HIST_RES;
        x_updated = true;
    } else if (right_idx != -1) {
        float exact_idx = refine_peak(x_hist, right_idx);
        calculated_x = TABLE_SIZE_X - ((exact_idx - HIST_CENTER) * HIST_RES);
        x_updated = true;
    }

    float calculated_y = prev_pose->y; 
    bool y_updated = false;
    
    // Pareil pour Y
    if (bottom_idx != -1) {
        float exact_idx = refine_peak(y_hist, bottom_idx);
        calculated_y = -(exact_idx - HIST_CENTER) * HIST_RES;
        y_updated = true;
    } else if (top_idx != -1) {
        float exact_idx = refine_peak(y_hist, top_idx);
        calculated_y = TABLE_SIZE_Y - ((exact_idx - HIST_CENTER) * HIST_RES);
        y_updated = true;
    }

    // Si on ne voit vraiment rien (gros blocage), on rejette proprement
    if (!x_updated && !y_updated && fabsf(angular_error) < 0.01f) {
        return result;
    }

    // step 6: Gating (Filtre anti-téléportation)
    // On vérifie que la correction ne propose pas un saut aberrant (> 300 mm)
    float dx = calculated_x - prev_pose->x;
    float dy = calculated_y - prev_pose->y;
    float dist_jump = sqrtf(dx*dx + dy*dy);
    
    if (dist_jump <= 300.0f) {
        result.x = calculated_x;
        result.y = calculated_y;
        
        // On renvoie l'angle de l'odométrie (erreur d_theta = 0)
        result.theta = prev_pose->theta; 
        
        result.valid = true;
    }

    return result;
}