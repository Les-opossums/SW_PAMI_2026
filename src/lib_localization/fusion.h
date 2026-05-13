#ifndef FUSION_H
#define FUSION_H

#define FUSION_GAIN_XY 0.15f
#define FUSION_GAIN_THETA 0.10f
// fusion.h
#define MAX_FUSION_JUMP 100.0f // Réduit à 100mm au lieu de 500mm pour être plus strict
#define FUSION_MAX_ERROR_THRESHOLD 150.0f // Si l'erreur > 150mm, on ignore totalement le LiDAR


void Fusion_Init(float start_x, float start_y, float start_theta_rad);

void Fusion_Predict(float vx_local, float vy_local, float v_angular_rad, float dt);

void Fusion_Correct(RobotPose lidar_meas);

RobotPose Fusion_GetState(void);

#endif // FUSION_H