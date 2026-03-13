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
    VelocityCommand rep = {goal_vx, goal_vy, 0.0f, false};

    if (scan == NULL || scan->index == 0) return rep;

    float g_norm = sqrtf(goal_vx * goal_vx + goal_vy * goal_vy);
    if (g_norm < 0.01f) return rep;

    float norm_gx = goal_vx / g_norm;
    float norm_gy = goal_vy / g_norm;

    float field_radius = 300.0f;
    float avoid_radius = 80.0f;

    // Vitesse minimale garantie pendant l'évitement actif
    // Empêche qu'une faible vitesse vers la goal rende la déviation imperceptible
    // À calibrer : 50% de ta vitesse nominale typique
    static const float MIN_AVOID_SPEED = 0.35f;

    float min_d = field_radius;
    float cross_sum = 0.0f;
    int found = 0;

    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;
        if (d < 10.0f || d >= field_radius) continue;

        float px = scan->points[i].x;
        float py = scan->points[i].y;

        float dot = px * norm_gx + py * norm_gy;
        if (dot < -30.0f) continue;

        cross_sum += norm_gx * py - norm_gy * px;

        if (d < min_d) {
            min_d = d;
            found = 1;
        }
    }

    static float swirl_sign    = 1.0f;
    static int   is_avoiding   = 0;
    static int   free_cycles   = 0;
    static float filtered_cross = 0.0f;
    static const int   EXIT_DELAY  = 25;
    // Seuil d'hystérésis : doit être >> bruit (~250 dans tes logs)
    // mais < signal d'un vrai nouvel obstacle (~3000 pour un mur)
    // → 800 est un bon compromis, ajuste si besoin
    static const float HYSTERESIS  = 800.0f;

    if (found) {
        free_cycles = 0;

        // Filtre passe-bas : lisse les oscillations rapides de cross_sum
        filtered_cross = filtered_cross * 0.75f + cross_sum * 0.25f;

        if (!is_avoiding) {
            // Fixe le signe UNE SEULE FOIS à l'entrée de l'évitement
            swirl_sign  = (filtered_cross >= 0.0f) ? -1.0f : 1.0f;
            is_avoiding = 1;
        } else {
            // Autorise le flip UNIQUEMENT si un obstacle est CLAIREMENT
            // de l'autre côté (nouveau mur, pas du bruit autour de 0)
            if (swirl_sign > 0.0f && filtered_cross > +HYSTERESIS) {
                swirl_sign = -1.0f;
            } else if (swirl_sign < 0.0f && filtered_cross < -HYSTERESIS) {
                swirl_sign = +1.0f;
            }
            // Sinon : signe verrouillé, pas d'oscillation
        }

        float t = (field_radius - min_d) / (field_radius - avoid_radius);
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        float deflection = swirl_sign * (M_PI * 0.60f) * t;

        // Vitesse effective : si le robot est presque à sa goal (vitesse faible),
        // on impose un minimum pour que la rotation ait un effet réel
        float effective_norm = g_norm;
        if (effective_norm < MIN_AVOID_SPEED && t > 0.15f) {
            effective_norm = MIN_AVOID_SPEED;
        }

        float eff_vx = norm_gx * effective_norm;
        float eff_vy = norm_gy * effective_norm;

        float cos_d = cosf(deflection);
        float sin_d = sinf(deflection);

        rep.vx = eff_vx * cos_d - eff_vy * sin_d;
        rep.vy = eff_vx * sin_d + eff_vy * cos_d;

        printf("DBG avoid: min_d=%.0f t=%.2f angle=%.1fdeg sign=%.0f raw=%.0f filt=%.0f\n",
               min_d, t, deflection * 180.0f / M_PI, swirl_sign, cross_sum, filtered_cross);
    } else {
        free_cycles++;
        if (free_cycles >= EXIT_DELAY) {
            is_avoiding     = 0;
            filtered_cross  = 0.0f; // Reset propre pour la prochaine rencontre
        }
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