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
float min_cluster_pts;
float cluster_tolerance;


// Internal state for the current goal
static Point2D current_goal = {0.0f, 0.0f};

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

    min_cluster_pts = MIN_CLUSTER_PTS;
    cluster_tolerance = CLUSTER_TOLERANCE;
}

// Fonction de filtrage pour éliminer le bruit (entretoises/reflets)
static bool is_valid_obstacle(const LD19DataPointHandler* scan, int index) {
    int neighbor_count = 1;
    float base_d = scan->points[index].distance;
    
    // On ignore les points physiquement impossibles (à l'intérieur du lidar/chassis)
    if (base_d < 60.0f) return false; 
    
    // On élargit la fenêtre de recherche (+2) pour tolérer 1 ou 2 points "ratés"
    int window = (int)min_cluster_pts + 2; 

    // Regarde devant
    for (int i = 1; i <= window; i++) {
        int idx = (index + i) % scan->index;
        float d = scan->points[idx].distance;
        if (d > 60.0f && fabsf(d - base_d) < cluster_tolerance) neighbor_count++;
    }
    // Regarde derrière
    for (int i = 1; i <= window; i++) {
        int idx = (index - i + scan->index) % scan->index;
        float d = scan->points[idx].distance;
        if (d > 60.0f && fabsf(d - base_d) < cluster_tolerance) neighbor_count++;
    }
    
    return neighbor_count >= (int)min_cluster_pts;
}

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

void Path_GetRepulsionVector(const LD19DataPointHandler* scan, float motion_angle_rad, RobotPose current_pose, float *rep_vx, float *rep_vy) {
    *rep_vx = 0.0f;
    *rep_vy = 0.0f;

    if (scan == NULL || scan->index == 0) return;

    float min_front_dist = d_max;
    float front_angle = 0.0f;
    bool front_found = false;

    float min_blind_dist = shield_max;
    float blind_angle = 0.0f;
    bool blind_found = false;

    float motion_angle_deg = motion_angle_rad * (180.0f / M_PI);

    // 1. RECHERCHE DES MENACES
    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;
        
        // --- CORRECTION MAJEURE ICI ---
        // ON NE FAIT PLUS "if (d <= d_min) continue;" !!!
        // On élimine seulement ce qui est DANS le rayon du robot (ex: 60mm).
        // Le filtre is_valid_obstacle se charge tout seul de supprimer les entretoises.
        if (d < 60.0f || !is_valid_obstacle(scan, i)) continue;

        float angle_deg = scan->points[i].angle;
        if (angle_deg > 180.0f) angle_deg -= 360.0f;
        
        float angle_rad = scan->points[i].angle * (M_PI / 180.0f);

        // --- CONSCIENCE DE LA TABLE ---
        float point_global_angle = current_pose.theta + angle_rad;
        float pt_x = current_pose.x + d * cosf(point_global_angle);
        float pt_y = current_pose.y + d * sinf(point_global_angle);

        // Assure-toi que table_size_x et table_size_y sont bien globales ou définies
        bool is_border = false;
        if (pt_x < BORDER_MARGIN || pt_x > (table_size_x - BORDER_MARGIN) ||
            pt_y < BORDER_MARGIN || pt_y > (table_size_y - BORDER_MARGIN)) {
            is_border = true; 
        }

        // --- CALCUL DE L'ÉCART AVEC LE MOUVEMENT ---
        float diff_angle = angle_deg - motion_angle_deg;
        while (diff_angle > 180.0f) diff_angle -= 360.0f;
        while (diff_angle < -180.0f) diff_angle += 360.0f;

        // --- SÉPARATION DES ZONES DYNAMIQUE ---
        // --- CORRECTION MAJEURE 2 : Élargissement du cône ---
        // Le cône passe de 35° à 65° pour ne pas "perdre de vue" l'obstacle pendant qu'on le contourne.
        if (diff_angle > -65.0f && diff_angle < 65.0f && !is_border) {
            if (d < min_front_dist) {
                min_front_dist = d; // Peut descendre sous d_min, c'est normal !
                front_angle = angle_rad; 
                front_found = true;
            }
        } else {
            if (d < min_blind_dist) {
                min_blind_dist = d;
                blind_angle = angle_rad;
                blind_found = true;
            }
        }
    }

    float sum_vx = 0.0f;
    float sum_vy = 0.0f;

    // 2. CALCUL DE LA FORCE AVANT
    if (front_found) {
        // Grâce à la disparition du "continue", si un obstacle s'approche à 50mm, 
        // penetration dépassera 1.0, mais la ligne ci-dessous (1.2f) la bloquera proprement au maximum !
        float penetration = (d_max - min_front_dist) / (d_max - d_min);
        if (penetration < 0.0f) penetration = 0.0f;
        if (penetration > 1.2f) penetration = 1.2f; 

        float force_mag = force_max * penetration;

        float r_x = -cosf(front_angle);
        float r_y = -sinf(front_angle);
        float t_x = -sinf(front_angle);
        float t_y =  cosf(front_angle);

        sum_vx += force_mag * (r_x + LATERAL_GAIN * t_x);
        sum_vy += force_mag * (r_y + LATERAL_GAIN * t_y);
    }

    // 3. CALCUL DU BOUCLIER 360°
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

    // FILTRE LISSAGE EXPONENTIEL
    static float filtered_vx = 0.0f;
    static float filtered_vy = 0.0f;
    const float ALPHA = 0.8f; // Astuce: Si le robot esquive de façon trop molle (retard), passe ça à 0.5 ou 0.6

    filtered_vx = ALPHA * sum_vx + (1.0f - ALPHA) * filtered_vx;
    filtered_vy = ALPHA * sum_vy + (1.0f - ALPHA) * filtered_vy;

    *rep_vx = filtered_vx;
    *rep_vy = filtered_vy;
}

// Adaptation de la fonction qui génère les consignes
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

    float motion_angle_rad = atan2f(F_att_y, F_att_x);

    float F_rep_x = 0.0f;
    float F_rep_y = 0.0f;
    
    // --- ON PASSE DESORMAIS LA POSE ACTUELLE ---
    Path_GetRepulsionVector(scan, motion_angle_rad, current_pose, &F_rep_x, &F_rep_y);

    cmd.vx = F_att_x + F_rep_x;
    cmd.vy = F_att_y + F_rep_y;
    
    limit_magnitude(&cmd.vx, &cmd.vy, PF_MAX_SPEED);

    cmd.omega = 0.0f; 

    return cmd;
}

void Set_pathfinding_parameters(float goal_tolerance, float max_speed, float max_rotation, float attractive_gain,
                                float dmin, float dmax, float fmax, float lat_gain, float shld_max, float f_shield) {
    pf_goal_tolerance = goal_tolerance;
    pf_max_speed = max_speed;
    pf_max_rotation = max_rotation;
    pf_attractive_gain = attractive_gain;

    d_min = dmin;
    d_max = dmax;
    force_max = fmax;
    lateral_gain = lat_gain;
    shield_max = shld_max;
    force_shield = f_shield;
}

uint8_t Set_Pathfinding_parameters_Cmd(void){
    uint32_t param;
    float valf;
    if (Get_Param_u32(&param))
        return 1;
    if (Get_Param_Float(&valf))
        return 1;

    switch(param) {
        case 0: pf_goal_tolerance = valf; printf("Pathfinding Parameter 'Goal_Tolerance' set to %.2f\n", valf);break;
        case 1: pf_max_speed = valf; printf("Pathfinding Parameter 'Max_Speed' set to %.2f\n", valf); break;
        case 2: pf_max_rotation = valf; printf("Pathfinding Parameter 'Max_Rotation' set to %.2f\n", valf); break;
        case 3: pf_attractive_gain = valf; printf("Pathfinding Parameter 'Attractive_Gain' set to %.2f\n", valf); break;
        case 4: d_min = valf; printf("Pathfinding Parameter 'D_min' set to %.2f\n", valf); break;
        case 5: d_max = valf; printf("Pathfinding Parameter 'D_max' set to %.2f\n", valf); break;
        case 6: force_max = valf; printf("Pathfinding Parameter 'Force_Max' set to %.2f\n", valf); break;
        case 7: lateral_gain = valf; printf("Pathfinding Parameter 'Lateral_Gain' set to %.2f\n", valf); break;
        case 8: shield_max = valf; printf("Pathfinding Parameter 'Shield_Max' set to %.2f\n", valf); break;
        case 9: force_shield = valf; printf("Pathfinding Parameter 'Force_Shield' set to %.2f\n", valf); break;
        case 10: min_cluster_pts = valf; printf("Pathfinding Parameter 'Min_Cluster_Pts' set to %.2f\n", valf); break;
        case 11: cluster_tolerance = valf; printf("Pathfinding Parameter 'Cluster_Tolerance' set to %.2f\n", valf); break;
        default: return 2; // Paramètre inconnu
    }
    return 0;
}

uint8_t Get_Pathfinding_parameters_Cmd(void){
    // 1. Ton log console classique
    printf("Pathfinding Parameters: Goal_Tolerance=%.2f, Max_Speed=%.2f, Max_Rotation=%.2f, Attractive_Gain=%.2f, D_min=%.2f, D_max=%.2f, Force_Max=%.2f, Lateral_Gain=%.2f, Shield_Max=%.2f, Force_Shield=%.2f, Min_Cluster_Pts=%.2f, Cluster_Tolerance=%.2f\n",
        (double)pf_goal_tolerance, (double)pf_max_speed, (double)pf_max_rotation, (double)pf_attractive_gain,
        (double)d_min, (double)d_max, (double)force_max, (double)lateral_gain, (double)shield_max, (double)force_shield, (double)min_cluster_pts, (double)cluster_tolerance);

    // 2. Formatage pour la page Web (séparé par des virgules pour un décodage facile en JS)
    char response[256];
    snprintf(response, sizeof(response), "PF_PARAMS:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
        (double)pf_goal_tolerance, (double)pf_max_speed, (double)pf_max_rotation, (double)pf_attractive_gain,
        (double)d_min, (double)d_max, (double)force_max, (double)lateral_gain, (double)shield_max, (double)force_shield, (double)min_cluster_pts, (double)cluster_tolerance);

    // 3. Envoi via lwIP (Assure-toi d'avoir accès à tcp_state global ici, ou passe-le en paramètre si besoin)
    // /!\ Le code ci-dessous est un exemple à adapter si tcp_state n'est pas global dans ton architecture.
    extern tcp_server_t *tcp_state; // Exemple si défini en global
    if (tcp_state && tcp_state->is_connected) {
        char ws_buf[300];
        uint64_t pack_len = WS_BuildPacket(ws_buf, sizeof(ws_buf), 
                                           WEBSOCKET_OPCODE_TEXT, 
                                           response, strlen(response), 0);
        tcp_server_send_data(tcp_state, (uint8_t*)ws_buf, pack_len);
    }
    
    return 0;
}