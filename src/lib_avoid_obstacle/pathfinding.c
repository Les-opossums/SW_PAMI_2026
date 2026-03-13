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

    // 1. calcul de la direction normalisé vers la cible
    float g_norm = sqrtf(goal_vx * goal_vx + goal_vy * goal_vy);
    if(g_norm < 0.005f) return rep; // onest arrivé pas de répulsion

    float gx = goal_vx / g_norm;
    float gy = goal_vy / g_norm;

    // vecteur latéral perpendiculaire à la direction vers la cible (pour le vortex)
    float lat_x = -gy;
    float lat_y = gx;

    int is_blocked = 0;
    float closest_proj = PF_AVOID_DIST;
    float block_perp = 0.0f; // savoir si l'obstacle bloquant est à gauche ou à droite de la trajectoire pour faire un vortex dans le bon sens

    float surv_x = 0.0f, surv_y = 0.0f;

    for (int i = 0; i < scan->index; i++) {
        float dist = scan->points[i].distance;
        
        // Ignorer qui est hors de portée ou bruit interne
        if (dist > PF_REPULSIVE_DIST || dist < 10.0f) continue;

        float obs_x = scan->points[i].y; // local x (devant)
        float obs_y = -scan->points[i].x; // local y (gauche)
        
        // =========================================================
        // A. Analyse du couloir
        // =========================================================
        float proj = obs_x * gx + obs_y * gy; // distance de l'obstacle le long du chemin
        float perp = obs_x * lat_x + obs_y * lat_y; // decalage de 'obstacle par rapport à mon chemin

        if (proj > 0.0f && proj < PF_AVOID_DIST && fabsf(perp) < PF_ROBOT_RADIUS){
            is_blocked = 1;
            // on mémorise le point bloquant le plus proche
            if (proj< closest_proj){
                closest_proj = proj;
                block_perp = perp;
            }
        }

        // =========================================================
        // B. bulle de survie (ne pas toucher les murs)
        // =========================================================
        if(dist < PF_SURVIVAL_DIST){
            float safe_dist = (dist < 20.0f) ? 20.0f : dist;
            float force = 800.0f * (1.0f / safe_dist - 1.0f / PF_SURVIVAL_DIST); // force très forte à courte distance, décroissante avec la distance
            if (force > 300.0f) force = 300.0f;

            surv_x += (-obs_x / safe_dist) * force;
            surv_y += (-obs_y / safe_dist) * force;
        }
    }

    // =========================================================
    // C. calcul du décalage latéral
    // =========================================================
    static float escape_sign = 1.0f; // pour alterner le sens d'évitement en cas de blocage
    static int avoiding_state = 0;
    float avoid_x = 0.0f, avoid_y = 0.0f;

    if (is_blocked){
        if(avoiding_state == 0){
            escape_sign = (block_perp > 0.0F) ? -1.0f : 1.0f;
            avoiding_state = 1;
        }
        float force_ratio = 1.0f - (closest_proj / PF_AVOID_DIST);
        float current_avoid_force = PF_AVOID_FORCE * (0.4f + 0.6f * force_ratio);

        avoid_x = lat_x * escape_sign * current_avoid_force;
        avoid_y = lat_y * escape_sign * current_avoid_force;

        float brake_force = PF_MAX_SPEED * force_ratio;
        avoid_x += -gx * brake_force;
        avoid_y += -gy * brake_force;
    } else {
        avoiding_state = 0;
    }

    rep.vx = avoid_x + surv_x;
    rep.vy = avoid_y + surv_y;
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