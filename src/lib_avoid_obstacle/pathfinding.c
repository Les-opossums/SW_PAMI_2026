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
    if (g_norm < 0.01f) return rep; 
    
    float norm_gx = goal_vx / g_norm;
    float norm_gy = goal_vy / g_norm;

    // ==================================================
    // paramétrage couloir
    // ==================================================
    float robot_radius = 50.0f;
    float margin = 30.0f;
    float stop_distance = robot_radius + margin;
    float anticipation_dist = 250.0f; 
    float corridor_width = robot_radius + margin;

    float max_brake_factor = 0.0f;
    float max_lat_force = 0.0f;

    static float swirl_sign = 1.0f;
    static int is_avoiding = 0;

    int threat_found = 0;
    float threat_cross_sum = 0.0f;

    float shield_x = 0.0f;
    float shield_y = 0.0f;

    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;

        if(d < 10.0f ) continue; // Ignorer les points très proches (bruit)
        if (d < 25.0f) d = 25.0f; // Éviter les forces extrêmes dues à des mesures très proches

        float px = scan->points[i].x;
        float py = -scan->points[i].y;

        // 1. champ de survie (sécurité physique absolue autour du robot)
        if (d < robot_radius + margin){
            float force = 2.0f * ((robot_radius + margin) - d);
            shield_x += (-px / d) * force;
            shield_y += (-py / d) * force;
        }

        // 2. projection dans le couloir de navigation
        float dot = (px * norm_gx) + (py * norm_gy);
        float cross = norm_gx * py - norm_gy * px;

        // 3. analyse du danger
        if (dot > 0.0f && dot < anticipation_dist){
            if (fabsf(cross) < corridor_width){
                threat_found = 1;
                threat_cross_sum += cross;

                
                float braking_factor = 0.0f;
                if (dot < stop_distance) braking_factor = 1.0f;
                else braking_factor = (anticipation_dist - dot) / (anticipation_dist - stop_distance);
                if(braking_factor > max_brake_factor) max_brake_factor = braking_factor;

                float lat = 0.0f;
                if(dot <= stop_distance) lat = 50.0f;
                else lat = 50.0f * (anticipation_dist - dot) / (anticipation_dist - stop_distance);
                if (lat > max_lat_force) max_lat_force = lat;
            }
        }
    }

    limit_magnitude(&shield_x, &shield_y, 50.0f);

    // ==================================================
    // application des forces fluides
    // ==================================================
    if (threat_found){
        if (is_avoiding == 0){
            if (threat_cross_sum > 0) swirl_sign = 1.0f;
            else swirl_sign = -1.0f;
            is_avoiding = 1;
        }

        // 1. freinage
        float F_brake_x = -norm_gx * max_brake_factor;
        float F_brake_y = -norm_gy * max_brake_factor;

        // 2. poussée latérale
        float lat_dir_x = -norm_gy * swirl_sign; // Perpendiculaire à la direction du but
        float lat_dir_y = norm_gx * swirl_sign;

        float F_lat_x = lat_dir_x * max_lat_force;
        float F_lat_y = lat_dir_y * max_lat_force;

        rep.vx = F_brake_x + F_lat_x + shield_x;
        rep.vy = F_brake_y + F_lat_y + shield_y;
    }else{
        is_avoiding = 0;
        rep.vx = shield_x;
        rep.vy = shield_y;
    }
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