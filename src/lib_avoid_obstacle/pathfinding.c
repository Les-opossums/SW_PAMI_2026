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

    float min_dist_front = PF_MIN_DIST;
    float obs_x = 0.0f;
    float obs_y = 0.0f;
    int threat_found = 0;

    float survival_radius = PF_SURVIVAL_DIST;
    float shield_x = 0.0f;
    float shield_y = 0.0f;

    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;

        if(d < 10.0f ) continue; // Ignorer les points très proches (bruit)

        if (d < 25.0f) d = 25.0f; // Éviter les forces extrêmes dues à des mesures très proches

        float px = scan->points[i].y;
        float py = -scan->points[i].x;

        float dot = (px * goal_vx) + (py * goal_vy);
        // ==================================================
        // A. SURVIE : est ce le point le plus proche autour du robot ?
        // ==================================================
        if (d < survival_radius){
            if(dot < -20.0f){
                float force = 4.0f * (survival_radius - d);

                shield_x += (-px / d) * force;
                shield_y += (-py / d) * force;
            }
        }

        // ==================================================
        // B. Navigation : est ce le point le plus proche dans la direction du but ?
        // ==================================================
        if (d < min_dist_front){
            if(dot < -10.0f){
                if(d < min_dist_front){
                    min_dist_front = d;
                    obs_x = px;
                    obs_y = py;
                    threat_found = 1;
                }   
            }
        }
    }

    limit_magnitude(&shield_x, &shield_y, PF_MAX_REPULSION);

    // ==================================================
    // FORCE DE CONTOURNEMENT
    // ==================================================

    static float swirl_sign = 1.0f;
    static int is_avoiding = 0;

    float F_brake_x = 0.0f;
    float F_brake_y = 0.0f;

    float F_tan_x = 0.0f;
    float F_tan_y = 0.0f;

    if(threat_found){
        float force_mag = PF_FORCE_MAG * (PF_AVOID_DIST - min_dist_front);
        if (force_mag > PF_MAX_REPULSION) force_mag = PF_MAX_REPULSION;
        
        float rx = -obs_x / min_dist_front;
        float ry = -obs_y / min_dist_front;

        F_brake_x = rx * force_mag;
        F_brake_y = ry * force_mag;

        if (is_avoiding == 0){
            float cross = goal_vx * ry - goal_vy * rx;
            swirl_sign = (cross >= 0.0f) ? -1.0f : 1.0f;
            is_avoiding = 1;
        }

        F_tan_x = -ry * swirl_sign * force_mag * 0.2f;
        F_tan_y =  rx * swirl_sign * force_mag * 0.2f;
    }else{
        is_avoiding = 0;
    }

    rep.vx = F_brake_x + F_tan_x + shield_x;
    rep.vy = F_brake_y + F_tan_y + shield_y;

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