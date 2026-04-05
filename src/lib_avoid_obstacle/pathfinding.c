#include "../PAMI_2026.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

vhf_parameters_t vfh_params;


// Internal state for the current goal
static Point2D current_goal = {0.0f, 0.0f};

void init_pathfinding_parameters(void) {
    vfh_params.sectors = VFH_SECTORS;
    vfh_params.robot_radius = VFH_ROBOT_RADIUS;
    vfh_params.margin = VFH_MARGIN;
    vfh_params.max_obstacle_dist = VFH_MAX_OBSTACLE_DIST;
    vfh_params.min_obstacle_dist = VFH_MIN_OBSTACLE_DIST;
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


VelocityCommand Path_ComputeVelocity(RobotPose current_pose, const LD19DataPointHandler* scan, float desired_speed) {
    VelocityCommand cmd = {0.0f, 0.0f, 0.0f, false};

    // 1. Calcul de la cible en global
    float dx_global = current_goal.x - current_pose.x;
    float dy_global = current_goal.y - current_pose.y;
    float distance_to_goal = sqrtf(dx_global*dx_global + dy_global*dy_global);

    // 2. Passage de la cible dans le repère local du robot
    float cos_theta = cosf(current_pose.theta);
    float sin_theta = sinf(current_pose.theta);
    float dx_local =  dx_global * cos_theta + dy_global * sin_theta;
    float dy_local = -dx_global * sin_theta + dy_global * cos_theta;

    // Angle de la cible dans le repère du robot
    float target_angle_rad = atan2f(dy_local, dx_local);
    float target_angle_deg = target_angle_rad * 180.0f / M_PI;
    while (target_angle_deg < 0.0f) target_angle_deg += 360.0f;

    // =========================================================
    // ETAPE A : CRÉATION DE L'HISTOGRAMME POLAIRE (VFH)
    // =========================================================
    bool blocked_sectors[VFH_SECTORS] = {false};
    float safe_distance = VFH_ROBOT_RADIUS + VFH_MARGIN;

    // --- ZONES MORTES (ENTRETOISES) ---
    // Angles des entretoises à masquer (en degrés)
    const float BLIND_SPOT_1 = 60.0f;
    const float BLIND_SPOT_2 = 180.0f;
    const float BLIND_SPOT_3 = 240.0f; // (A vérifier si ce n'est pas 300° selon ta CAO)
    
    // Tolérance : On ignore tous les points à +/- 12° autour de l'entretoise
    const float BLIND_SPOT_TOLERANCE = 12.0f; 

    float min_front_dist = VFH_MAX_OBSTACLE_DIST; 

    if (scan != NULL && scan->index > 0) {
        for (int i = 0; i < scan->index; i++) {
            float d = scan->points[i].distance;
            float angle_deg = scan->points[i].angle;

            // 1. Normalisation de l'angle strictement entre 0 et 360°
            float a = angle_deg;
            while (a < 0.0f) a += 360.0f;
            while (a >= 360.0f) a -= 360.0f;

            // 2. FILTRAGE DES ENTRETOISES
            bool is_blind_spot = false;
            // Comme les angles sont loin de 0/360, un simple fabsf() suffit pour tester l'écart
            if (fabsf(a - BLIND_SPOT_1) < BLIND_SPOT_TOLERANCE ||
                fabsf(a - BLIND_SPOT_2) < BLIND_SPOT_TOLERANCE ||
                fabsf(a - BLIND_SPOT_3) < BLIND_SPOT_TOLERANCE) {
                is_blind_spot = true;
            }

            // Si le point tape dans l'entretoise ou un fantôme, on le supprime (on passe au suivant)
            if (is_blind_spot) {
                continue; 
            }

            // 3. Suite du code normal VFH
            float diff_to_target = fabsf(angle_deg - target_angle_deg);
            if (diff_to_target > 180.0f) diff_to_target = 360.0f - diff_to_target;

            // On applique le filtre de distance globale (VFH_MIN et VFH_MAX) + Cône avant
            if (d > VFH_MIN_OBSTACLE_DIST && d < VFH_MAX_OBSTACLE_DIST && diff_to_target < 100.0f) {
                
                if (d < min_front_dist) {
                    min_front_dist = d;
                }

                int center_sector = (int)(a / (360.0f / VFH_SECTORS)) % VFH_SECTORS;

                // Limitation du ratio pour éviter l'effet "mur plat" de très près
                float ratio = safe_distance / d;
                if (ratio > 0.85f) ratio = 0.85f; 
                float enlargement_rad = asinf(ratio); 

                int sector_spread = (int)((enlargement_rad * 180.0f / M_PI) / (360.0f / VFH_SECTORS)) + 1;

                for (int s = -sector_spread; s <= sector_spread; s++) {
                    int idx = (center_sector + s) % VFH_SECTORS;
                    if (idx < 0) idx += VFH_SECTORS; 
                    blocked_sectors[idx] = true;
                }
            }
        }
    }

    // =========================================================
    // ETAPE B : RECHERCHE DE LA MEILLEURE DIRECTION LIBRE
    // =========================================================
    static int previous_best_sector = -1; // Mémoire de l'ancienne décision
    
    int target_sector = (int)(target_angle_deg / (360.0f / VFH_SECTORS)) % VFH_SECTORS;
    int best_sector = -1;
    float best_cost = 999999.0f;

    // POIDS DE DÉCISION (Tu pourras les ajuster)
    const float WEIGHT_TARGET = 1.0f;  // Envie d'aller vers la cible
    const float WEIGHT_PREVIOUS = 0.0f; // Envie de garder la même direction qu'avant (Anti-oscillation)

    for (int i = 0; i < VFH_SECTORS; i++) {
        if (!blocked_sectors[i]) {
            
            // 1. Écart avec le cap de la cible
            int diff_target = abs(i - target_sector);
            if (diff_target > VFH_SECTORS / 2) diff_target = VFH_SECTORS - diff_target;

            // 2. Écart avec la décision précédente
            int diff_prev = 0;
            if (previous_best_sector != -1) {
                diff_prev = abs(i - previous_best_sector);
                if (diff_prev > VFH_SECTORS / 2) diff_prev = VFH_SECTORS - diff_prev;
            }

            // Calcul du coût : Plus le coût est bas, meilleure est la trajectoire
            float cost = (diff_target * WEIGHT_TARGET) + (diff_prev * WEIGHT_PREVIOUS);

            if (cost < best_cost) {
                best_cost = cost;
                best_sector = i;
            }
        }
    }
    
    // On sauvegarde la décision pour le prochain tour de boucle
    previous_best_sector = best_sector;

    // =========================================================
    // ETAPE C : GÉNÉRATION DE LA COMMANDE DE VITESSE
    // =========================================================
    float raw_vx = 0.0f;
    float raw_vy = 0.0f;

    if (best_sector != -1) {
        float chosen_angle_deg = best_sector * (360.0f / VFH_SECTORS) + ((360.0f / VFH_SECTORS) / 2.0f);
        float chosen_angle_rad = chosen_angle_deg * M_PI / 180.0f;

        // --- NOUVEAU : Ralentissement proportionnel à l'effort d'esquive ---
        // On calcule de combien de degrés le robot a dû dévier par rapport à sa cible
        float angle_deviation = fabsf(chosen_angle_deg - target_angle_deg);
        if (angle_deviation > 180.0f) angle_deviation = 360.0f - angle_deviation;
        
        // Calcul du facteur de vitesse (0.3 à 1.0)
        // 0° de déviation = 100% de la vitesse
        // 90° de déviation = 30% de la vitesse
        float speed_factor = 1.0f - (angle_deviation / 90.0f) * 0.7f;
        
        // Sécurités pour ne jamais reculer ni s'arrêter complètement
        if (speed_factor < 0.3f) speed_factor = 0.3f; 
        if (speed_factor > 1.0f) speed_factor = 1.0f;

        // Application de la vitesse
        float final_speed = desired_speed * speed_factor;

        raw_vx = final_speed * cosf(chosen_angle_rad);
        raw_vy = final_speed * sinf(chosen_angle_rad);
    } else {
        // Le robot est 100% encerclé, on coupe la vitesse
        raw_vx = 0.0f;
        raw_vy = 0.0f;
    }

    // =========================================================
    // ETAPE D : LISSAGE DES CONSIGNES (Filtre Passe-Bas)
    // Empêche le robot de vibrer si la décision oscille entre 2 secteurs
    // =========================================================
    static float filtered_vx = 0.0f;
    static float filtered_vy = 0.0f;
    const float ALPHA = 0.8f; // 1.0 = pas de filtre, 0.1 = très lisse mais réagit lentement

    filtered_vx = ALPHA * raw_vx + (1.0f - ALPHA) * filtered_vx;
    filtered_vy = ALPHA * raw_vy + (1.0f - ALPHA) * filtered_vy;

    cmd.vx = filtered_vx;
    cmd.vy = filtered_vy;
    cmd.omega = 0.0f; // Asservissement angulaire géré indépendamment

    return cmd;
}

void Set_pathfinding_parameters(vhf_parameters_t new_params) {
    vfh_params = new_params;        
}

uint8_t Set_Pathfinding_parameters_Cmd(void){
    uint32_t param;
    float valf;
    if (Get_Param_u32(&param))
        return 1;
    if (Get_Param_Float(&valf))
        return 1;

    switch(param) {
        case 0: vfh_params.sectors = valf; printf("Pathfinding Parameter 'Sectors' set to %.2f\n", valf); break;
        case 1: vfh_params.robot_radius = valf; printf("Pathfinding Parameter 'Robot_Radius' set to %.2f\n", valf); break;
        case 2: vfh_params.margin = valf; printf("Pathfinding Parameter 'Margin' set to %.2f\n", valf); break;
        case 3: vfh_params.max_obstacle_dist = valf; printf("Pathfinding Parameter 'Max_Obstacle_Dist' set to %.2f\n", valf); break;
        case 4: vfh_params.min_obstacle_dist = valf; printf("Pathfinding Parameter 'Min_Obstacle_Dist' set to %.2f\n", valf); break;
        default: return 2; // Paramètre inconnu
    }
    return 0;
}

uint8_t Get_Pathfinding_parameters_Cmd(void){
    // 1. Ton log console classique
    printf("Pathfinding Parameters: Sectors=%.2f, Robot_Radius=%.2f, Margin=%.2f, Max_Obstacle_Dist=%.2f, Min_Obstacle_Dist=%.2f\n",
        (double)vfh_params.sectors, (double)vfh_params.robot_radius, (double)vfh_params.margin, (double)vfh_params.max_obstacle_dist, (double)vfh_params.min_obstacle_dist);

    // 2. Formatage pour la page Web (séparé par des virgules pour un décodage facile en JS)
    char response[256];
    snprintf(response, sizeof(response), "PF_PARAMS:%.2f,%.2f,%.2f,%.2f,%.2f\n",
        (double)vfh_params.sectors, (double)vfh_params.robot_radius, (double)vfh_params.margin, (double)vfh_params.max_obstacle_dist, (double)vfh_params.min_obstacle_dist);

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