#include "../PAMI_2026.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

float pf_goal_tolerance;
float pf_max_speed;
float pf_max_rotation;
float pf_attractive_gain;
float d_min;
float d_max;
float force_max;
float lateral_gain;
float shield_max;
float force_shield;

void init_pathfinding_parameters(void) {
    pf_goal_tolerance = PF_GOAL_TOLERANCE;
    pf_max_speed = PF_MAX_SPEED;
    pf_max_rotation = PF_MAX_ROTATION;
    pf_attractive_gain = PF_ATTRACTIVE_GAIN;

    d_min = D_MIN;
    d_max = D_MAX;
    force_max = FORCE_MAX;
    lateral_gain = LATERAL_GAIN;
    shield_max = SHIELD_MAX;
    force_shield = FORCE_SHIELD;
}

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

// Modifions la signature pour accepter l'angle de consigne (motion_angle_rad)
void Path_GetRepulsionVector(const LD19DataPointHandler* scan, float motion_angle_rad, float *rep_vx, float *rep_vy) {
    *rep_vx = 0.0f;
    *rep_vy = 0.0f;

    if (scan == NULL || scan->index == 0) return;

    // Variables pour l'avant dynamique
    float min_front_dist = d_max;
    float front_angle = 0.0f;
    bool front_found = false;

    // Variables pour les côtés et l'arrière
    float min_blind_dist = shield_max;
    float blind_angle = 0.0f;
    bool blind_found = false;

    // --- NOUVEAU : Conversion de l'angle de mouvement en degrés ---
    float motion_angle_deg = motion_angle_rad * (180.0f / M_PI);

    // 1. RECHERCHE DES MENACES
    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;
        float angle_deg = scan->points[i].angle;

        if (angle_deg > 180.0f) angle_deg -= 360.0f;

        // --- NOUVEAU : Calcul de l'écart entre le point Lidar et la trajectoire ---
        float diff_angle = angle_deg - motion_angle_deg;
        // On normalise cet écart pour qu'il reste entre -180° et +180°
        while (diff_angle > 180.0f) diff_angle -= 360.0f;
        while (diff_angle < -180.0f) diff_angle += 360.0f;

        // --- SÉPARATION DES ZONES DYNAMIQUE ---
        // Le cône tactique "suit" désormais la direction vers laquelle le robot se déplace !
        if (diff_angle > -35.0f && diff_angle < 35.0f) {
            // Zone Tactique : Dans l'axe du mouvement
            if (d > 0.0f && d < min_front_dist) {
                min_front_dist = d;
                front_angle = scan->points[i].angle * (M_PI / 180.0f); // IMPORTANT: On garde l'angle VRAI pour la physique !
                front_found = true;
            }
        } else {
            // Zone de Survie : Partout ailleurs
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
        float penetration = (d_max - min_front_dist) / (d_max - d_min);
        if (penetration < 0.0f) penetration = 0.0f;
        if (penetration > 1.2f) penetration = 1.2f; 

        float force_mag = force_max * penetration;

        float r_x = -cosf(front_angle);
        float r_y = -sinf(front_angle);
        float t_x = -sinf(front_angle);
        float t_y = cosf(front_angle);

        sum_vx += force_mag * (r_x + LATERAL_GAIN * t_x);
        sum_vy += force_mag * (r_y + LATERAL_GAIN * t_y);
    }

    // 3. CALCUL DU BOUCLIER 360° (Répulsion pure uniquement)
    if (blind_found) {
        float penetration = (shield_max - min_blind_dist) / (shield_max - d_min);
        if (penetration < 0.0f) penetration = 0.0f;
        if (penetration > 1.2f) penetration = 1.2f; 

        float force_mag = force_shield * penetration;

        float r_x = -cosf(blind_angle);
        float r_y = -sinf(blind_angle);

        sum_vx += force_mag * r_x;
        sum_vy += force_mag * r_y;
    }

    limit_magnitude(&sum_vx, &sum_vy, force_max * 1.2f);

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

    float cos_theta = cosf(current_pose.theta);
    float sin_theta = sinf(current_pose.theta);
    float dx_local =  dx_global * cos_theta + dy_global * sin_theta;
    float dy_local = -dx_global * sin_theta + dy_global * cos_theta;

    float F_att_x = dx_local * PF_ATTRACTIVE_GAIN;
    float F_att_y = dy_local * PF_ATTRACTIVE_GAIN;
    limit_magnitude(&F_att_x, &F_att_y, PF_MAX_SPEED);

    // --- NOUVEAU : Quel est l'angle de notre vecteur d'attraction local ? ---
    // atan2f(Y, X) nous donne la direction précise vers laquelle le robot VOUDRAIT aller.
    float motion_angle_rad = atan2f(F_att_y, F_att_x);

    float F_rep_x = 0.0f;
    float F_rep_y = 0.0f;
    // On passe cet angle magique à notre fonction de détection
    Path_GetRepulsionVector(scan, motion_angle_rad, &F_rep_x, &F_rep_y);

    cmd.vx = F_att_x + F_rep_x;
    cmd.vy = F_att_y + F_rep_y;
    
    limit_magnitude(&cmd.vx, &cmd.vy, PF_MAX_SPEED);

    cmd.omega = 0.0f; 

    return cmd;
}