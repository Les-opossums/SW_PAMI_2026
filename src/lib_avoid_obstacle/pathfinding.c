#include "../PAMI_2026.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Internal state for the current goal
static Point2D current_goal = {0.0f, 0.0f};

// Helper: Limit magnitude of a vector
static void limit_magnitude(float* x, float* y, float max_val) {
    float mag_sq = (*x * *x) + (*y * *y);
    if (mag_sq > (max_val * max_val)) {
        float mag = sqrtf(mag_sq);
        *x = (*x / mag) * max_val;
        *y = (*y / mag) * max_val;
    }
}

void Path_Init(void) {
    current_goal.x = 0.0f;
    current_goal.y = 0.0f;
}

void Path_SetGoal(float x, float y) {
    current_goal.x = x;
    current_goal.y = y;
}


VelocityCommand Path_GetRepulsionVector(const LD19DataPointHandler* scan, float goal_vx, float goal_vy) {
    VelocityCommand rep = {0.0f, 0.0f, 0.0f, false};

    float g_norm = sqrtf(goal_vx * goal_vx + goal_vy * goal_vy);
    if (g_norm > 0.01f) {
        goal_vx /= g_norm;
        goal_vy /= g_norm;
    }else{
        return rep; 
    }

    float min_dist = PF_MIN_DIST;
    float obs_x = 0.0f;
    float obs_y = 0.0f;
    int threat_found = 0;

    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;
        if(d > 20.0f && d < min_dist){
            float px = scan->points[i].y;
            float py = -scan->points[i].x;

            float dot = (px * goal_vx) + (py * goal_vy);
            if(dot  < -20.0f){
                min_dist = d;
                obs_x = px;
                obs_y = py;
                threat_found = 1;
            }

        }
    }

    static float swirl_sign = 1.0f;
    static int is_avoiding = 0;

    if(!threat_found){
        is_avoiding =0;
        return rep;
    }

    float force_mag= PF_FORCE_MAG * (PF_MIN_DIST - min_dist);
    if (force_mag > 500.0f) force_mag = 500.0f;

    float rx = -obs_x /min_dist;
    float ry = -obs_y /min_dist;

    float F_rep_x = rx * force_mag;
    float F_rep_y = ry * force_mag;

    if (is_avoiding == 0){
        float cross = goal_vx * ry - goal_vy * rx;
        swirl_sign = (cross >= 0.0f) ? -1.0f : 1.0f;
        is_avoiding = 1;
    }
    
    float F_tan_x = -ry * swirl_sign * force_mag * 0.3f;
    float F_tan_y = rx * swirl_sign * force_mag *0.3f;

    rep.vx = F_rep_x + F_tan_x;
    rep.vy = F_rep_y + F_tan_y;

    return rep;

}

VelocityCommand Path_ComputeVelocity(RobotPose current_pose, const LD19DataPointHandler* scan) {
    VelocityCommand cmd = {0.0f, 0.0f, 0.0f, false};

    // --- 1. Force Attractive (Vers la cible) ---
    float dx_global = current_goal.x - current_pose.x;
    float dy_global = current_goal.y - current_pose.y;
    float distance_to_goal = sqrtf(dx_global * dx_global + dy_global * dy_global);

    if (distance_to_goal < PF_GOAL_TOLERANCE) {
        cmd.reached = true;
        return cmd; 
    }

    float cos_theta = cosf(current_pose.theta);
    float sin_theta = sinf(current_pose.theta);
    float dx_local = dx_global * cos_theta + dy_global * sin_theta;
    float dy_local = -dx_global * sin_theta + dy_global * cos_theta;

    float F_att_x = dx_local * PF_ATTRACTIVE_GAIN;
    float F_att_y = dy_local * PF_ATTRACTIVE_GAIN;

    // CORRECTION MAJEURE : On bride l'attraction ! 
    // L'envie d'aller vers la cible ne doit jamais dépasser la vitesse max.
    // Ainsi, une répulsion forte d'un mur proche pourra la surpasser.
    limit_magnitude(&F_att_x, &F_att_y, PF_MAX_SPEED);

    // --- 2. Force Répulsive & Vortex (Contre les obstacles) ---
    VelocityCommand rep = Path_GetRepulsionVector(scan, dx_local, dy_local);

    // --- 3. Somme des forces ---
    float F_total_x = F_att_x + rep.vx;
    float F_total_y = F_att_y + rep.vy;

    // --- 4. Conversion en vitesse de consigne ---
    // C'est seulement à la toute fin qu'on protège les moteurs pour ne pas demander 
    // une consigne absurde s'il fuit très vite.
    limit_magnitude(&F_total_x, &F_total_y, PF_MAX_SPEED);

    cmd.vx = F_total_x;
    cmd.vy = F_total_y;
    cmd.omega = 0.0f; 

    return cmd;
}