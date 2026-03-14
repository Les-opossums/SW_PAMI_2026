#include "../PAMI_2026.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Internal state for the current goal
static Point2D current_goal = {0.0f, 0.0f};

void Path_Init(void) {
    current_goal.x = 0.0f;
    current_goal.y = 0.0f;
}

void Path_SetGoal(float x, float y) {
    current_goal.x = x;
    current_goal.y = y;
}

// Helper: Limit magnitude of a vector
static void limit_magnitude(float* x, float* y, float max_val) {
    float mag_sq = (*x * *x) + (*y * *y);
    if (mag_sq > (max_val * max_val)) {
        float mag = sqrtf(mag_sq);
        *x = (*x / mag) * max_val;
        *y = (*y / mag) * max_val;
    }
}

void Path_GetRepulsionVector(const LD19DataPointHandler* scan, float *rep_vx, float *rep_vy) {
    *rep_vx = 0.0f;
    *rep_vy = 0.0f;

    if (scan == NULL || scan->index == 0) return;

    // =========================================================
    // PARAMÈTRES
    // =========================================================
    const float D_MIN = 60.0f; 

    // 1. Paramètres du Cône Avant (Tactique)
    const float D_MAX = 160.0f; 
    const float FORCE_MAX = 300.0f; 
    const float LATERAL_GAIN = 1.0f; 

    // 2. Paramètres du Bouclier Angles Morts (Survie)
    const float SHIELD_MAX = 100.0f;  // S'active seulement très près !
    const float FORCE_SHIELD = 400.0f; // Force de poussée latérale/arrière

    // Variables pour l'avant
    float min_front_dist = D_MAX;
    float front_angle = 0.0f;
    bool front_found = false;

    // Variables pour les côtés et l'arrière
    float min_blind_dist = SHIELD_MAX;
    float blind_angle = 0.0f;
    bool blind_found = false;

    // 1. RECHERCHE DES MENACES
    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;
        float angle_deg = scan->points[i].angle;

        // On normalise l'angle entre -180° et +180°
        if (angle_deg > 180.0f) angle_deg -= 360.0f;

        // --- SÉPARATION DES ZONES ---
        if (angle_deg > -35.0f && angle_deg < 35.0f) {
            // Zone Tactique : Cône avant
            if (d > 0.0f && d < min_front_dist) {
                min_front_dist = d;
                front_angle = scan->points[i].angle * (M_PI / 180.0f);
                front_found = true;
            }
        } else {
            // Zone de Survie : Angles morts (Tout le reste)
            if (d > 0.0f && d < min_blind_dist) {
                min_blind_dist = d;
                blind_angle = scan->points[i].angle * (M_PI / 180.0f);
                blind_found = true;
            }
        }
    }

    float sum_vx = 0.0f;
    float sum_vy = 0.0f;

    // 2. CALCUL DE LA FORCE AVANT (Répulsion + Glissade)
    if (front_found) {
        float penetration = (D_MAX - min_front_dist) / (D_MAX - D_MIN);
        if (penetration < 0.0f) penetration = 0.0f;
        if (penetration > 1.2f) penetration = 1.2f; 

        float force_mag = FORCE_MAX * penetration;

        float r_x = -cosf(front_angle);
        float r_y = -sinf(front_angle);
        float t_x = -sinf(front_angle);
        float t_y = cosf(front_angle);

        sum_vx += force_mag * (r_x + LATERAL_GAIN * t_x);
        sum_vy += force_mag * (r_y + LATERAL_GAIN * t_y);
    }

    // 3. CALCUL DU BOUCLIER 360° (Répulsion pure uniquement)
    if (blind_found) {
        float penetration = (SHIELD_MAX - min_blind_dist) / (SHIELD_MAX - D_MIN);
        if (penetration < 0.0f) penetration = 0.0f;
        if (penetration > 1.2f) penetration = 1.2f; 

        float force_mag = FORCE_SHIELD * penetration;

        // Pas de glissade latérale ici, on veut juste repousser le mur !
        float r_x = -cosf(blind_angle);
        float r_y = -sinf(blind_angle);

        sum_vx += force_mag * r_x;
        sum_vy += force_mag * r_y;
    }

    // On limite par sécurité
    // On autorise un peu plus de vitesse globale si les deux forces s'additionnent
    limit_magnitude(&sum_vx, &sum_vy, FORCE_MAX * 1.2f);

    // =========================================================
    // LE FILTRE (Lissage pour éviter qu'il tremble)
    // =========================================================
    static float filtered_vx = 0.0f;
    static float filtered_vy = 0.0f;
    
    const float ALPHA = 0.3f; 

    filtered_vx = ALPHA * sum_vx + (1.0f - ALPHA) * filtered_vx;
    filtered_vy = ALPHA * sum_vy + (1.0f - ALPHA) * filtered_vy;

    *rep_vx = filtered_vx;
    *rep_vy = filtered_vy;
}

// --- TON ARCHITECTURE CONSERVÉE ET SIMPLIFIÉE ---
VelocityCommand Path_ComputeVelocity(RobotPose current_pose, const LD19DataPointHandler* scan) {
    VelocityCommand cmd = {0.0f, 0.0f, 0.0f, false};

    float dx_global = current_goal.x - current_pose.x;
    float dy_global = current_goal.y - current_pose.y;
    float distance_to_goal = sqrtf(dx_global*dx_global + dy_global*dy_global);

    if (distance_to_goal < PF_GOAL_TOLERANCE) {
        cmd.reached = true;
        return cmd;
    }

    // Passage du vecteur d'attraction dans le repère local du robot
    float cos_theta = cosf(current_pose.theta);
    float sin_theta = sinf(current_pose.theta);
    float dx_local =  dx_global * cos_theta + dy_global * sin_theta;
    float dy_local = -dx_global * sin_theta + dy_global * cos_theta;

    // Vecteur attraction (force qui tire vers la cible)
    float F_att_x = dx_local * PF_ATTRACTIVE_GAIN;
    float F_att_y = dy_local * PF_ATTRACTIVE_GAIN;
    limit_magnitude(&F_att_x, &F_att_y, PF_MAX_SPEED);

    // Vecteur de répulsion (force qui repousse des murs)
    float F_rep_x = 0.0f;
    float F_rep_y = 0.0f;
    Path_GetRepulsionVector(scan, &F_rep_x, &F_rep_y);

    // L'APF : C'est la somme des deux forces !
    cmd.vx = F_att_x + F_rep_x;
    cmd.vy = F_att_y + F_rep_y;
    
    // On peut appliquer une dernière limite globale pour protéger les moteurs
    limit_magnitude(&cmd.vx, &cmd.vy, PF_MAX_SPEED);

    cmd.omega = 0.0f; // Asservissement angulaire géré ailleurs j'imagine

    return cmd;
}