#ifndef PATHFINDING_H
#define PATHFINDING_H

// =========================================================
// PARAMÈTRES DE LA NAVIGATION PAR SECTEURS
// =========================================================
#define VFH_SECTORS 72                 // 360° divisé par 72 = 5° par secteur
#define VFH_ROBOT_RADIUS 75.0f         // Rayon physique de ton robot (en mm)
#define VFH_MARGIN 40.0f               // Marge de sécurité autour du robot (en mm)
#define VFH_MAX_OBSTACLE_DIST 350.0f   // Distance au-delà de laquelle on ignore les obstacles (mm)
#define VFH_MIN_OBSTACLE_DIST 80.0f    // Distance en dessous de laquelle on ignore les obstacles (mm)
typedef struct {
    float sectors;
    float robot_radius;
    float margin;
    float max_obstacle_dist;
    float min_obstacle_dist;
} vhf_parameters_t;

extern vhf_parameters_t vfh_params;

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
VelocityCommand Path_ComputeVelocity(RobotPose current_pose, const LD19DataPointHandler* scan, float desired_speed);

void init_pathfinding_parameters(void);
void Set_pathfinding_parameters(vhf_parameters_t new_params);
uint8_t Set_Pathfinding_parameters_Cmd(void);
uint8_t Get_Pathfinding_parameters_Cmd(void);

#endif // PATHFINDING_H