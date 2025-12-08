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

void Fusion_Predict(float v_linear, float v_angular_rad, float dt) {
    if (!state.valid) return;

    // update orientation
    state.theta = normalize_angle(state.theta + v_angular_rad * dt);

    // update position
    state.x += v_linear * cosf(state.theta) * dt;
    state.y += v_linear * sinf(state.theta) * dt;
}

void Fusion_Correct(RobotPose lidar_meas) {
    if (!lidar_meas.valid) return;

    float dx = lidar_meas.x - state.x;
    float dy = lidar_meas.y - state.y;
    float dist_sq = dx * dx + dy * dy;

    if (dist_sq > MAX_FUSION_JUMP * MAX_FUSION_JUMP) {
        return;
    }

    state.x += dx * FUSION_GAIN_XY;
    state.y += dy * FUSION_GAIN_XY;

    float d_theta = angle_diff_rad(lidar_meas.theta, state.theta);
    state.theta += d_theta * FUSION_GAIN_THETA;

    state.theta = normalize_angle(state.theta);
}

RobotPose Fusion_GetState(void) {
    return state;
}
