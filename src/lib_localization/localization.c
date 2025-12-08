#include "../PAMI_2026.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_TWO_PI
#define M_TWO_PI 6.28318530717958647692 
#endif

// -- internal constants --
#define HIST_RES 20.0f // 20 mm per bin
#define HIST_SIZE 300 // range coverage
#define HIST_CENTER (HIST_SIZE / 2) // center index of histogram
#define MIN_WALL_PTS 5 // minimum number of points to consider a wall valid

// Helper
static float normalize_angle(float angle) {
    while (angle < 0.0f) angle += M_TWO_PI;
    while (angle >= M_TWO_PI) angle -= M_TWO_PI;
    return angle;
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

    // step 1 : determine heading (orientation)
    //get the local grid alignment
    float grid_angle = find_grid_alignement_rad(scan);

    // ambiguity resolution :
    // The grid_angle doesn't know if we are at , 90 deg, 180 deg, etc.
    // we use previous pose to resolve this ambiguity
    float step = M_PI / 2.0f; // 90 degrees
    float base_angle = roundf((prev_pose->theta - grid_angle) / step) * step;

    // combine base + offset to get robot orientation
    float global_theta = normalize_angle(base_angle + grid_angle);

    // step 2 : rotate points to align with global frame
    float cos_theta = cosf(-global_theta);
    float sin_theta = sinf(-global_theta);

    // step 3 : build histograms for X and Y positions
    uint16_t x_hist[HIST_SIZE] = {0};
    uint16_t y_hist[HIST_SIZE] = {0};

    for (int i = 0; i < scan->index; i++) {
        // ignore invalid or out of range points
        if(scan->points[i].distance < 50 || scan->points[i].distance > 3500) continue;

        // rotate point
        float x_rot = scan->points[i].x * cos_theta - scan->points[i].y * sin_theta;
        float y_rot = scan->points[i].x * sin_theta + scan->points[i].y * cos_theta;

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
    // we scan from the edges of the array, the first bin with enough points is the wall
    // this effectively ignors noise inside the table area
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
    // the "span" is the distance between walls in mm 
    float span_x = (right_idx - left_idx) * HIST_RES;
    float span_y = (top_idx - bottom_idx) * HIST_RES;

    // calculate distance from robot to the left and bottom walls
    float dist_to_left = (left_idx - HIST_CENTER) * HIST_RES;
    float dist_to_bottom = (bottom_idx - HIST_CENTER) * HIST_RES;

    // step 6: compute robot position in global frame
    //CASE A : we check if measured width matches defined wideth and measured length defined length
    if (fabsf(span_x - TABLE_SIZE_X) < LOC_TOLERANCE_MM &&
        fabsf(span_y - TABLE_SIZE_Y) < LOC_TOLERANCE_MM) {
        // position is distance to left and bottom walls
        result.x = -dist_to_left;
        result.y = -dist_to_bottom;
        result.theta = global_theta;
        result.valid = true;
        return result;
    }
    // CASE B : rotated 90 degrees
    else if (fabsf(span_x - TABLE_SIZE_Y) < LOC_TOLERANCE_MM &&
             fabsf(span_y - TABLE_SIZE_X) < LOC_TOLERANCE_MM) {
        // position is distance to bottom and left walls swapped
        result.x = -dist_to_bottom;
        result.y = -dist_to_left;
        result.theta = normalize_angle(global_theta + M_PI / 2.0f);
        result.valid = true;
        return result;
    }
}