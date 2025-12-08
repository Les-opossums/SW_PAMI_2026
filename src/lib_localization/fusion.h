#ifndef FUSION_H
#define FUSION_H

#define FUSION_GAIN_XY 0.15f
#define FUSION_GAIN_THETA 0.10f
#define MAX_FUSION_JUMP 500.0f // in mm

void Fusion_Init(float start_x, float start_y, float start_theta_rad);

void Fusion_Predict(float v_linear, float v_angular_rad, float dt);

void Fusion_Correct(RobotPose lidar_meas);

RobotPose Fusion_GetState(void);

#endif // FUSION_H