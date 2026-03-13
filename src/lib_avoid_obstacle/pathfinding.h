#ifndef PATHFINDING_H
#define PATHFINDING_H

// #define DEBUG_ORIENTATION 

// --- Tuning Parameters ---
#define PF_GOAL_TOLERANCE     50.0f   
#define PF_MAX_SPEED          300.0f  
#define PF_MAX_ROTATION       2.0f    

#define PF_ATTRACTIVE_GAIN    1.5f    

#define PF_AVOID_DIST 200.0f


#define PF_MIN_DIST 220.0f
#define PF_SURVIVAL_DIST 80.0f
#define PF_FORCE_MAG 6.0f
#define PF_MAX_REPULSION 200.0f 


typedef struct {
    float vx;       // Linear Velocity X (mm/s) in Robot Frame
    float vy;       // Linear Velocity Y (mm/s) in Robot Frame
    float omega;    // Angular Velocity (rad/s)
    bool reached;   // True if goal is reached
} VelocityCommand;

typedef struct {
    float x;
    float y;
} Point2D;

/**
 * @brief Initializes the pathfinding system
 */
void Path_Init(void);

/**
 * @brief Sets the global target goal
 * @param x Global X coordinate (mm)
 * @param y Global Y coordinate (mm)
 */
void Path_SetGoal(float x, float y);

/**
 * @brief Computes the next velocity command based on Potential Fields
 * * @param current_pose The robot's current estimated pose (Global Frame)
 * @param scan The latest LIDAR scan data (Robot Frame)
 * @return VelocityCommand (vx, vy, omega) in Robot Local Frame
 */
VelocityCommand Path_ComputeVelocity(RobotPose current_pose, const LD19DataPointHandler* scan);

VelocityCommand Path_GetRepulsionVector(const LD19DataPointHandler* scan, float current_vx, float current_vy);

#endif // PATHFINDING_H