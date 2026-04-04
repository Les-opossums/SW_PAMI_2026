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
    float front_diff_angle = 0.0f; // NOUVEAU : Pour savoir de quel côté est l'obstacle
    bool front_found = false;

    float min_blind_dist = shield_max;
    float blind_angle = 0.0f;
    bool blind_found = false;

    float motion_angle_deg = motion_angle_rad * (180.0f / M_PI);

    // 1. RECHERCHE DES MENACES
    for (int i = 0; i < scan->index; i++) {
        float d = scan->points[i].distance;
        
        // Rayon physique du robot : on ignore ce qui est "à l'intérieur" de lui
        if (d < 60.0f || !is_valid_obstacle(scan, i)) continue;

        float angle_deg = scan->points[i].angle;
        if (angle_deg > 180.0f) angle_deg -= 360.0f;
        
        float angle_rad = scan->points[i].angle * (M_PI / 180.0f);

        // --- CONSCIENCE DE LA TABLE ---
        float point_global_angle = current_pose.theta + angle_rad;
        float pt_x = current_pose.x + d * cosf(point_global_angle);
        float pt_y = current_pose.y + d * sinf(point_global_angle);

        bool is_border = false;
        if (pt_x < BORDER_MARGIN || pt_x > (3000.0f - BORDER_MARGIN) ||
            pt_y < BORDER_MARGIN || pt_y > (2000.0f - BORDER_MARGIN)) {
            is_border = true; 
        }

        // --- CALCUL DE L'ÉCART AVEC LE MOUVEMENT ---
        float diff_angle = angle_deg - motion_angle_deg;
        while (diff_angle > 180.0f) diff_angle -= 360.0f;
        while (diff_angle < -180.0f) diff_angle += 360.0f;

        // --- SÉPARATION DES ZONES DYNAMIQUE ---
        // Cône de 65° pour un suivi d'obstacle stable
        if (diff_angle > -65.0f && diff_angle < 65.0f && !is_border) {
            if (d < min_front_dist) {
                min_front_dist = d;
                front_angle = angle_rad; 
                front_diff_angle = diff_angle; // On sauvegarde l'angle relatif
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

    // 2. CALCUL DE LA FORCE AVANT (Avec Esquive Intelligente)
    if (front_found) {
        float penetration = (d_max - min_front_dist) / (d_max - d_min);
        if (penetration < 0.0f) penetration = 0.0f;
        if (penetration > 1.0f) penetration = 1.0f; // Bloqué à 100% max

        float force_mag = force_max * penetration;

        // --- NOUVEAU : CHOIX DU CÔTÉ D'ESQUIVE ---
        // Si l'obstacle est à gauche de ma trajectoire (>0), je glisse à droite (-1)
        // Si l'obstacle est à droite (<0), je glisse à gauche (+1)
        float dodge_sign = (front_diff_angle > 0.0f) ? -1.0f : 1.0f;

        float r_x = -cosf(front_angle);
        float r_y = -sinf(front_angle);
        
        // Vecteur tangentiel (orienté du bon côté !)
        float t_x = -sinf(front_angle) * dodge_sign;
        float t_y =  cosf(front_angle) * dodge_sign;

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

    // FILTRE LISSAGE EXPONENTIEL (Réactif)
    static float filtered_vx = 0.0f;
    static float filtered_vy = 0.0f;
    const float ALPHA = 0.6f; // Un lissage doux (0.6) pour enlever les vibrations du Lidar sans être en retard

    filtered_vx = ALPHA * sum_vx + (1.0f - ALPHA) * filtered_vx;
    filtered_vy = ALPHA * sum_vy + (1.0f - ALPHA) * filtered_vy;

    *rep_vx = filtered_vx;
    *rep_vy = filtered_vy;
}

// =========================================================
// PARAMÈTRES DE LA NAVIGATION PAR SECTEURS
// =========================================================
#define NUM_SECTORS 72            // 72 secteurs = résolution de 5 degrés (très précis)
#define SECTOR_ANGLE (360.0f / NUM_SECTORS)
#define ROBOT_RADIUS 120.0f       // Rayon de ton robot (en mm) + une petite marge
#define AVOID_DIST 400.0f         // À partir de combien de mm on prend en compte l'obstacle
// #define BORDER_MARGIN 60.0f       // Marge des murs de la table

VelocityCommand Path_ComputeVelocity(RobotPose current_pose, const LD19DataPointHandler* scan) {
    VelocityCommand cmd = {0.0f, 0.0f, 0.0f, false};

    float dx_global = current_goal.x - current_pose.x;
    float dy_global = current_goal.y - current_pose.y;
    float distance_to_goal = sqrtf(dx_global*dx_global + dy_global*dy_global);

    if (distance_to_goal < PF_GOAL_TOLERANCE) {
        cmd.reached = true;
        return cmd;
    }

    // 1. Calcul de l'angle parfait vers la cible (en LOCAL par rapport au robot)
    float cos_theta = cosf(current_pose.theta);
    float sin_theta = sinf(current_pose.theta);
    float dx_local =  dx_global * cos_theta + dy_global * sin_theta;
    float dy_local = -dx_global * sin_theta + dy_global * cos_theta;

    float target_angle_rad = atan2f(dy_local, dx_local);
    float target_angle_deg = target_angle_rad * (180.0f / M_PI);
    if (target_angle_deg < 0.0f) target_angle_deg += 360.0f;

    // 2. Construction de l'histogramme (false = Voie Libre, true = Obstacle)
    bool blocked_sectors[NUM_SECTORS] = {false};

    if (scan != NULL && scan->index > 0) {
        for (int i = 0; i < scan->index; i++) {
            float d = scan->points[i].distance;
            
            // On ignore le châssis, les entretoises, et ce qui est trop loin
            if (d < 60.0f || d > AVOID_DIST || !is_valid_obstacle(scan, i)) continue;

            float angle_deg = scan->points[i].angle;
            if (angle_deg < 0.0f) angle_deg += 360.0f;

            // --- CONSCIENCE DE LA TABLE ---
            float point_global_angle = current_pose.theta + (angle_deg * (M_PI / 180.0f));
            float pt_x = current_pose.x + d * cosf(point_global_angle);
            float pt_y = current_pose.y + d * sinf(point_global_angle);

            // On ne bloque pas les secteurs qui regardent les murs de la table !
            if (pt_x < BORDER_MARGIN || pt_x > (3000.0f - BORDER_MARGIN) ||
                pt_y < BORDER_MARGIN || pt_y > (2000.0f - BORDER_MARGIN)) {
                continue; 
            }

            // --- ON BLOQUE LE SECTEUR ---
            int center_sector = (int)(angle_deg / SECTOR_ANGLE) % NUM_SECTORS;
            blocked_sectors[center_sector] = true;

            // --- MAGIE GEOMÉTRIQUE : On élargit l'obstacle de la taille du robot ---
            // Plus l'obstacle est proche, plus il bloque de secteurs autour de lui.
            float safe_ratio = ROBOT_RADIUS / d;
            if (safe_ratio > 1.0f) safe_ratio = 1.0f;
            float angular_width_deg = asinf(safe_ratio) * (180.0f / M_PI);
            
            // Nombre de secteurs additionnels à bloquer à gauche et à droite
            int num_extra_sectors = (int)(angular_width_deg / SECTOR_ANGLE) + 1;
            
            for (int s = 1; s <= num_extra_sectors; s++) {
                int left_sec = (center_sector + s) % NUM_SECTORS;
                int right_sec = (center_sector - s + NUM_SECTORS) % NUM_SECTORS;
                blocked_sectors[left_sec] = true;
                blocked_sectors[right_sec] = true;
            }
        }
    }

    // 3. Choix du meilleur chemin (Le secteur libre le plus proche de la cible)
    int best_sector = -1;
    int target_sector = (int)(target_angle_deg / SECTOR_ANGLE) % NUM_SECTORS;
    
    if (!blocked_sectors[target_sector]) {
        // La voie royale est libre !
        best_sector = target_sector; 
    } else {
        // La voie est bloquée, on scrute à gauche et à droite progressivement
        for (int offset = 1; offset < NUM_SECTORS / 2; offset++) {
            int sec_plus = (target_sector + offset) % NUM_SECTORS;
            int sec_minus = (target_sector - offset + NUM_SECTORS) % NUM_SECTORS;
            
            // Le premier secteur libre trouvé devient notre nouvelle trajectoire
            if (!blocked_sectors[sec_plus]) {
                best_sector = sec_plus;
                break;
            }
            if (!blocked_sectors[sec_minus]) {
                best_sector = sec_minus;
                break;
            }
        }
    }

    // 4. Génération de la commande cinématique
    if (best_sector != -1) {
        float chosen_angle_deg = best_sector * SECTOR_ANGLE + (SECTOR_ANGLE / 2.0f);
        float chosen_angle_rad = chosen_angle_deg * (M_PI / 180.0f);

        // Vitesse proportionnelle pour s'arrêter en douceur sur la cible
        float current_speed_max = distance_to_goal * PF_ATTRACTIVE_GAIN;
        if (current_speed_max > PF_MAX_SPEED) current_speed_max = PF_MAX_SPEED;

        // On projette cette vitesse sur le vecteur local choisi
        float target_vx = current_speed_max * cosf(chosen_angle_rad);
        float target_vy = current_speed_max * sinf(chosen_angle_rad);

        // Léger filtre pour éviter les à-coups si le lidar hésite entre 2 secteurs
        static float filtered_vx = 0.0f;
        static float filtered_vy = 0.0f;
        float alpha = 0.6f; // 1.0 = aucune inertie, 0.1 = très mou

        filtered_vx = alpha * target_vx + (1.0f - alpha) * filtered_vx;
        filtered_vy = alpha * target_vy + (1.0f - alpha) * filtered_vy;

        cmd.vx = filtered_vx;
        cmd.vy = filtered_vy;
    } else {
        // Mode panique : tous les secteurs sont bloqués (le robot est encerclé)
        cmd.vx = 0.0f;
        cmd.vy = 0.0f;
    }

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