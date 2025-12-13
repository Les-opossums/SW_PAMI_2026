#include "../PAMI_2026.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Internal state for the current goal
static Point2D current_goal = {0.0f, 0.0f};

// Helper: Limit magnitude of a vector
static void limit_magnitude(float* x, float* y, float max_val) {
    float mag_sq = (*x * *x) + (*y * *y);
    if (mag_sq > (max_val * max_val)) {
        float mag = sqrtf(mag_sq);
        *x = (*x / mag) * max_val;
        *y = (*y / mag) * max_val;
    }
}

// Helper: Normalize angle to -PI to +PI
static float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void Path_Init(void) {
    current_goal.x = 0.0f;
    current_goal.y = 0.0f;
}

void Path_SetGoal(float x, float y) {
    current_goal.x = x;
    current_goal.y = y;
}

VelocityCommand Path_Compute(RobotPose current_pose, const LD19DataPointHandler* scan) {
    VelocityCommand cmd = {0};

    // --- 1. Transform Goal to Robot Local Frame ---
    // We do this so we can sum forces in the robot's own X/Y perspective.
    float dx_global = current_goal.x - current_pose.x;
    float dy_global = current_goal.y - current_pose.y;

    // Rotate vector by -theta to get into local frame
    float cos_th = cosf(current_pose.theta);
    float sin_th = sinf(current_pose.theta);
    
    // Local Goal Vector
    float goal_x_local = dx_global * cos_th + dy_global * sin_th;
    float goal_y_local = -dx_global * sin_th + dy_global * cos_th;

    // Check distance to goal
    float dist_to_goal = hypotf(goal_x_local, goal_y_local);
    
    if (dist_to_goal < PF_GOAL_TOLERANCE) {
        cmd.vx = 0;
        cmd.vy = 0;
        cmd.omega = 0;
        cmd.reached = true;
        return cmd;
    }

    // --- 2. Calculate Attractive Force (Pull to goal) ---
    float F_att_x = PF_ATTRACTIVE_GAIN * goal_x_local;
    float F_att_y = PF_ATTRACTIVE_GAIN * goal_y_local;
    
    // Clamp attractive force so it doesn't dominate at long ranges
    limit_magnitude(&F_att_x, &F_att_y, PF_MAX_SPEED);

    // --- 3. Calculate Repulsive Force (Push from obstacles) ---
    float F_rep_x = 0.0f;
    float F_rep_y = 0.0f;

    // Optimization: Skip points to save CPU? 
    // RP2350 is fast enough for all 1200 points, but step=2 is safe if needed.
    int step = 1; 

    for (int i = 0; i < scan->index; i += step) {
        float ox = scan->points[i].x;
        float oy = scan->points[i].y;
        float dist = scan->points[i].distance; // Already in mm

        // Ignore noise (too close) or far away points
        if (dist < 50.0f || dist > PF_REPULSIVE_DIST) continue;

        // Vector from Obstacle -> Robot (which is at 0,0 locally)
        // So vector is just (-ox, -oy)
        // Normalize: u_x = -ox / dist
        
        // Repulsive strength: proportional to (1/dist - 1/R)^2
        // We simplify calculation for speed:
        // Force = Gain * (R - dist) / (dist * dist)
        float rep_factor = (PF_REPULSIVE_DIST - dist);
        // Using squared distance in denominator for stronger reaction when very close
        float force_mag = PF_REPULSIVE_GAIN * (rep_factor / (dist * dist));

        // Standard Repulsion Vector (push away)
        float rx = (-ox / dist) * force_mag;
        float ry = (-oy / dist) * force_mag;

        // Vortex Field (Tangent)
        // To avoid local minima, we add a force perpendicular to the obstacle.
        // If the goal is to the "left" of the obstacle, rotate force left, else right.
        // Simple heuristic: Rotate 90 degrees based on cross product sign with goal.
        
        // Cross product z = (goal_x * oy - goal_y * ox)
        // If z > 0, goal is "left" of obstacle vector -> swirl Clockwise? 
        // Let's just add a constant swirl to the right for consistency or adaptive.
        // Adaptive Swirl:
        float cross_prod = goal_x_local * oy - goal_y_local * ox;
        float swirl_sign = (cross_prod > 0) ? 1.0f : -1.0f;

        // Rotate repulsion vector 90 degrees: (x, y) -> (-y, x) * sign
        float vx = -ry * swirl_sign * PF_VORTEX_GAIN;
        float vy =  rx * swirl_sign * PF_VORTEX_GAIN;

        F_rep_x += (rx + vx);
        F_rep_y += (ry + vy);
    }

    // --- 4. Sum Forces ---
    float F_total_x = F_att_x + F_rep_x;
    float F_total_y = F_att_y + F_rep_y;

    // --- 5. Convert Force to Velocity Command ---
    // For a holonomic robot, Force vector ~~ Velocity vector (mass = 1)
    
    // Limit Max Linear Speed
    limit_magnitude(&F_total_x, &F_total_y, PF_MAX_SPEED);

    cmd.vx = F_total_x;
    cmd.vy = F_total_y;

    // --- 6. Angular Velocity (Face the direction of motion) ---
    // Holonomic robots can move in X/Y while facing any theta. 
    // Usually, it's safer to face the direction of movement or face the goal.
    // Strategy: Face the Goal.
    float target_heading = atan2f(goal_y_local, goal_x_local); // Angle relative to robot front
    
    // Proportional control for rotation
    cmd.omega = 2.0f * target_heading; 
    
    // Clamp rotation
    if (cmd.omega > PF_MAX_ROTATION) cmd.omega = PF_MAX_ROTATION;
    if (cmd.omega < -PF_MAX_ROTATION) cmd.omega = -PF_MAX_ROTATION;

    cmd.reached = false;
    return cmd;
}