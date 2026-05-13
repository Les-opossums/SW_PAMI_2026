#include "../PAMI_2026.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_TWO_PI
#define M_TWO_PI 6.28318530717958647692 
#endif

static RobotPose state;

static float normalize_angle(float angle) {
    while (angle < 0.0f) angle += M_TWO_PI;
    while (angle >= M_TWO_PI) angle -= M_TWO_PI;
    return angle;
}

static float angle_diff_rad(float target, float current) {
    float diff = target - current;
    while (diff < -M_PI) diff += M_TWO_PI;
    while (diff > M_PI) diff -= M_TWO_PI;
    return diff;
}

void Fusion_Init(float start_x, float start_y, float start_theta_rad) {
    state.x = start_x;
    state.y = start_y;
    state.theta = normalize_angle(start_theta_rad);
    state.valid = true;
}

void Fusion_Predict(float vx_local, float vy_local, float v_angular_rad, float dt) {if (!state.valid) return;

    // 1. Mise à jour de l'orientation
    state.theta = normalize_angle(state.theta + v_angular_rad * dt);

    // 2. Passage des vitesses locales (repère robot) au repère global
    float cos_t = cosf(state.theta);
    float sin_t = sinf(state.theta);

    // 3. Mise à jour de la position
    state.x += (vx_local * cos_t - vy_local * sin_t) * dt;
    state.y += (vx_local * sin_t + vy_local * cos_t) * dt;
}

// fusion.c
void Fusion_Correct(RobotPose lidar_meas) {
    if (!lidar_meas.valid || !state.valid) return;

    // Calcul de la distance entre la position actuelle (odo) et la mesure LiDAR
    float dx = lidar_meas.x - state.x;
    float dy = lidar_meas.y - state.y;
    float dist_error = sqrtf(dx*dx + dy*dy);

    // Si la mesure est trop loin (> 150mm), on considère que c'est une erreur de lecture
    // Le PAMI continuera alors en "full odométrie"
    if (dist_error > FUSION_MAX_ERROR_THRESHOLD) {
        printf("Fusion_Correct: Ignoring LiDAR correction due to large error (%.1f mm)\n", dist_error);
        return; 
    }

    // On applique les gains de fusion seulement si l'erreur est raisonnable
    state.x += dx * FUSION_GAIN_XY;
    state.y += dy * FUSION_GAIN_XY;

    float dtheta = angle_diff_rad(lidar_meas.theta, state.theta);
    state.theta = normalize_angle(state.theta + dtheta * FUSION_GAIN_THETA);
}

RobotPose Fusion_GetState(void) {
    return state;
}
