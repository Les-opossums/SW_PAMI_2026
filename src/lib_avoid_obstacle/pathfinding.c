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

// --- NOUVELLE FONCTION APF PURE ET SANS MÉMOIRE ---
void Path_GetRepulsionVector(const LD19DataPointHandler* scan, float *rep_vx, float *rep_vy) {
    *rep_vx = 0.0f;
    *rep_vy = 0.0f;

    if (scan == NULL || scan->index == 0) return;

    float sum_vx = 0.0f;
    float sum_vy = 0.0f;

    // Paramètres à régler (tu peux les mettre en #define si tu préfères)
    const float INFLUENCE_RADIUS = 300.0f; // mm
    const float ROBOT_RADIUS     = 100.0f;  // mm
    const float REPULSION_GAIN   = 250000.0f; 
    const float MAX_REP_SPEED    = 300.0f; // mm/s

    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;

        if (d > 0.0f && d <= INFLUENCE_RADIUS) {
            if (d < ROBOT_RADIUS) d = ROBOT_RADIUS; // Sécurité mathématique

            float force_magnitude = REPULSION_GAIN * ((1.0f / d) - (1.0f / INFLUENCE_RADIUS)) / (d * d);
            
            float angle_rad = scan->points[i].angle * (M_PI / 180.0f);

            // Répulsion (on s'éloigne de l'obstacle)
            sum_vx -= force_magnitude * cosf(angle_rad);
            sum_vy -= force_magnitude * sinf(angle_rad);
        }
    }

    limit_magnitude(&sum_vx, &sum_vy, MAX_REP_SPEED);

    static float filtered_vx = 0.0f;
    static float filtered_vy = 0.0f;
    
    // Le coefficient ALPHA (entre 0.0 et 1.0)
    // 1.0 = Aucune filtration (réaction instantanée, très nerveux)
    // 0.1 = Très filtré (réaction lente et très douce)
    const float ALPHA = 0.15f; 

    // Si on a un scan vide, on laisse le filtre retomber doucement vers 0
    if (scan == NULL || scan->index == 0) {
        sum_vx = 0.0f;
        sum_vy = 0.0f;
    }

    // Application du filtre de lissage
    filtered_vx = ALPHA * sum_vx + (1.0f - ALPHA) * filtered_vx;
    filtered_vy = ALPHA * sum_vy + (1.0f - ALPHA) * filtered_vy;

    // On renvoie les valeurs lissées !
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