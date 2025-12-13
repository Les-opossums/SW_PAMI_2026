#ifndef PATHFINDING_H
#define PATHFINDING_H

// --- Tuning Parameters ---
// Adjust these to change robot behavior
#define PF_GOAL_TOLERANCE     50.0f   // [mm] Stop if closer than this
#define PF_MAX_SPEED          300.0f  // [mm/s] Maximum linear speed
#define PF_MAX_ROTATION       2.0f    // [rad/s] Maximum angular speed

#define PF_ATTRACTIVE_GAIN    1.5f    // Pull strength towards goal
#define PF_REPULSIVE_DIST     800.0f  // [mm] Obstacles further than this are ignored
#define PF_REPULSIVE_GAIN     4000.0f // Push strength away from obstacles
#define PF_VORTEX_GAIN        0.6f    // [0.0 - 1.0] How much we "slide" around obstacles vs bounce back

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
VelocityCommand Path_Compute(RobotPose current_pose, const LD19DataPointHandler* scan);

#endif // PATHFINDING_H