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

    float norm_gx = 0.0f;
    float norm_gy = 0.0f;

    if (g_norm > 0.01f) {
        norm_gx = goal_vx / g_norm;
        norm_gy = goal_vy / g_norm;
    }else{
        return rep; 
    }

    float min_dist_front = PF_AVOID_DIST;
    float obs_x = 0.0f;
    float obs_y = 0.0f;
    int threat_found = 0;

    float danger_left = 0.0f;
    float danger_right = 0.0f;

    float survival_radius = PF_SURVIVAL_DIST;
    float shield_x = 0.0f;
    float shield_y = 0.0f;

    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;

        if(d < 10.0f ) continue; // Ignorer les points très proches (bruit)

        if (d < 25.0f) d = 25.0f; // Éviter les forces extrêmes dues à des mesures très proches

        float px = scan->points[i].x;
        float py = -scan->points[i].y;

        float dot = (px * norm_gx) + (py * norm_gy);
        float cross_path = norm_gx * py - norm_gy * px;


        #ifdef DEBUG_ORIENTATION
            // Affichage pour debug orientation
            if(d > 100.0f && d < 400.0f){
                static int print_cpt = 0;
                if(print_cpt++ % 10 == 0){
                    printf("Lidar : X=%.1f, Y=%.1f | CIBLE : gx=%.2f, gy=%.2f | DOT=%.1f | CROSS=%.1f\n", px, py, norm_gx, norm_gy, dot, cross_path);
                }
            }
        #endif


        // ==================================================
        // SCANNER SPATIAL
        // ==================================================
        if (d < 600.0f && dot > -10.0f){
            float danger_weight = 1000.0f /d; // Plus proche = plus dangereux
            if(cross_path > 0){
                danger_left += danger_weight;
            }
            else{
                danger_right += danger_weight;
            }
        }

        // ==================================================
        // A. SURVIE : est ce le point le plus proche autour du robot ?
        // ==================================================
        if (d < survival_radius){
            float force = 2.0f * (survival_radius - d);

            shield_x += (-px / d) * force;
            shield_y += (-py / d) * force;
        }

        // ==================================================
        // B. Navigation : est ce le point le plus proche dans la direction du but ?
        // ==================================================
        if (d < min_dist_front){
            if(dot > 0.0f){
                min_dist_front = d;
                obs_x = px;
                obs_y = py;
                threat_found = 1;  
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
        
        float obs_cross_path = norm_gx * obs_y - norm_gy * obs_x;
        float lateral_factor = 1.0f - (fabsf(obs_cross_path) / 220.0f);
        if (lateral_factor < 0.0f) lateral_factor = 0.0f;
        
        force_mag *= lateral_factor;

        float rx = -obs_x / min_dist_front;
        float ry = -obs_y / min_dist_front;

        F_brake_x = rx * force_mag;
        F_brake_y = ry * force_mag;

        if (is_avoiding == 0){
            if (danger_left > danger_right) swirl_sign = 1.0f;
            else swirl_sign = -1.0f;
            is_avoiding = 1;
        }

        float tx = -ry * swirl_sign;
        float ty =  rx * swirl_sign;

        float centripetal_factor = 0.3f;

        F_tan_x = (tx - rx * centripetal_factor) * force_mag * 0.5f;
        F_tan_y = (ty - ry * centripetal_factor) * force_mag * 0.5f;
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

    float absolute_min_dist = 300.0f; //au dela de 30 cm, on n'est plus en danger immédiat
    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;
        if (d > 20.0f && d < absolute_min_dist) absolute_min_dist = d;
    }

    float dynamic_max_speed = PF_MAX_SPEED;
    if (absolute_min_dist < PF_AVOID_DIST) {
        dynamic_max_speed = PF_MAX_SPEED * (absolute_min_dist / PF_AVOID_DIST);
        if (dynamic_max_speed < 50.0f) dynamic_max_speed = 50.0f; // Vitesse minimale pour ne pas staller
    }
    // CORRECTION MAJEURE : On bride l'attraction ! 
    // L'envie d'aller vers la cible ne doit jamais dépasser la vitesse max.
    // Ainsi, une répulsion forte d'un mur proche pourra la surpasser.
    limit_magnitude(&F_att_x, &F_att_y, dynamic_max_speed);

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