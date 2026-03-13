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
    VelocityCommand rep = {goal_vx, goal_vy, 0.0f, false}; // Par défaut : on retourne le vecteur intact

    float g_norm = sqrtf(goal_vx * goal_vx + goal_vy * goal_vy);
    if (g_norm < 0.01f) return rep;

    float norm_gx = goal_vx / g_norm;
    float norm_gy = goal_vy / g_norm;

    float avoid_radius = 80.0f;
    float field_radius = 220.0f;

    float min_d = field_radius;
    float obs_x = 0.0f, obs_y = 0.0f;
    int   threat_found = 0;
    float cross_sum = 0.0f;

    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;
        if (d < 10.0f) continue;
        if (d < 25.0f) d = 25.0f;

        float px =  scan->points[i].x;
        float py = -scan->points[i].y;
        float dot = px * norm_gx + py * norm_gy;

        if (dot > -20.0f && d < field_radius) {
            if (d < min_d) {
                min_d = d; obs_x = px; obs_y = py;
                threat_found = 1;
            }
            cross_sum += norm_gx * py - norm_gy * px;
        }
    }

    static float swirl_sign = 1.0f;
    static int   is_avoiding = 0;
    static int   free_cycles = 0;
    static const int EXIT_DELAY = 15;

    if (threat_found) {
        free_cycles = 0;
        if (!is_avoiding) {
            swirl_sign  = (cross_sum > 0.0f) ? 1.0f : -1.0f;
            is_avoiding = 1;
        }

        // t_factor : 0 à la lisière du champ, 1 au contact de l'obstacle
        float t_factor = (field_radius - min_d) / (field_radius - avoid_radius);
        if (t_factor < 0.0f) t_factor = 0.0f;
        if (t_factor > 1.0f) t_factor = 1.0f;

        // Déviation max = 90° quand collé à l'obstacle, 0° à field_radius
        // On peut monter à (M_PI * 0.6f) si 90° ne suffit pas à contourner
        float deflection = swirl_sign * (M_PI_2)* 0.65 * t_factor;

        float cos_d = cosf(deflection);
        float sin_d = sinf(deflection);

        // Rotation du vecteur objectif — la norme est CONSERVÉE
        rep.vx = goal_vx * cos_d - goal_vy * sin_d;
        rep.vy = goal_vx * sin_d + goal_vy * cos_d;

    } else {
        free_cycles++;
        if (free_cycles >= EXIT_DELAY) {
            is_avoiding = 0;
        }
        // rep = {goal_vx, goal_vy} — déjà initialisé en haut
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