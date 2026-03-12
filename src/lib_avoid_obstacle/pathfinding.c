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

    float F_rep_x = 0.0f;
    float F_rep_y = 0.0f;

    // Variables pour isoler l'obstacle le plus menaçant devant (pour le vortex)
    float min_dist_front = PF_REPULSIVE_DIST;
    float closest_front_x = 0.0f;
    float closest_front_y = 0.0f;
    int obs_in_front = 0;

    for (int i = 0; i < scan->index; i++) {
        float dist = scan->points[i].distance;
        
        // Ignorer le bruit et ce qui est hors de portée
        if (dist < 50.0f || dist > PF_REPULSIVE_DIST) continue;

        // Repère local : x = Avant, y = Gauche
        float obs_x = scan->points[i].y; 
        float obs_y = -scan->points[i].x;

        // Vecteur unitaire fuyant le point
        float rx = -obs_x / dist;
        float ry = -obs_y / dist;

        // Calcul de la force pure. 
        float force = PF_REPULSIVE_GAIN / (dist * dist);
        if (force > 50.0f) force = 50.0f; // Limite de stabilité par point

        F_rep_x += rx * force;
        F_rep_y += ry * force;

        // Recherche du point le plus proche UNIQUEMENT devant le robot pour le vortex
        if (obs_x > 0.0f) {
            if (dist < min_dist_front) {
                min_dist_front = dist;
                closest_front_x = obs_x;
                closest_front_y = obs_y;
            }
            obs_in_front++;
        }
    }

    // =========================================================
    // Hystérésis (Mémoire du sens d'esquive)
    // =========================================================
    static float locked_swirl_sign = 1.0f;
    static int is_avoiding = 0;

    // S'il n'y a aucune force de répulsion, on reset
    if (F_rep_x == 0.0f && F_rep_y == 0.0f) {
        is_avoiding = 0; 
        return rep;
    }

    // =========================================================
    // Calcul du Vortex intelligent (Basé sur le point le plus proche)
    // =========================================================
    float vortex_x = 0.0f;
    float vortex_y = 0.0f;

    if (obs_in_front > 0) {
        // CORRECTION ICI : La force du vortex est proportionnelle à l'obstacle + bridée
        float base_force = PF_REPULSIVE_GAIN / (min_dist_front * min_dist_front);
        if (base_force > 50.0f) base_force = 50.0f; 
        float v_force = base_force * PF_VORTEX_GAIN;
        
        // Vecteur unitaire fuyant l'obstacle frontal le plus proche
        float rx = -closest_front_x / min_dist_front;
        float ry = -closest_front_y / min_dist_front;

        if (is_avoiding == 0) {
            // Produit vectoriel pour choisir le côté vers la cible
            float cross_prod = goal_vx * ry - goal_vy * rx;
            locked_swirl_sign = (cross_prod >= 0.0f) ? -1.0f : 1.0f;
            is_avoiding = 1;
        }

        // Rotation à 90 degrés du vecteur de fuite
        vortex_x = -ry * locked_swirl_sign * v_force;
        vortex_y =  rx * locked_swirl_sign * v_force;
    } else {
        // Plus d'obstacle devant, on désactive le vortex
        is_avoiding = 0;
    }

    // =========================================================
    // Somme finale (SANS PLAFOND POUR LAISSER LE MUR GAGNER)
    // =========================================================
    float total_x = F_rep_x + vortex_x;
    float total_y = F_rep_y + vortex_y;

    // SUPPRESSION du bridage MAX_TOTAL_SPEED ici. 
    // La répulsion a désormais le droit de valoir 1000 ou 2000 si on est collé au mur.

    rep.vx = total_x;
    rep.vy = total_y;

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