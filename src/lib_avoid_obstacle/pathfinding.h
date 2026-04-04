#ifndef PATHFINDING_H
#define PATHFINDING_H

// --- Tuning Parameters ---
#define PF_GOAL_TOLERANCE     50.0f   
#define PF_MAX_SPEED          300.0f  
#define PF_MAX_ROTATION       2.0f    
#define PF_ATTRACTIVE_GAIN    1.5f    


// =========================================================
// PARAMÈTRES
// =========================================================
#define D_MIN           60.0f

// 1. Paramètres du Cône Avant (Tactique)
#define D_MAX           160.0f
#define FORCE_MAX       300.0f 
#define LATERAL_GAIN    1.0f 

// 2. Paramètres du Bouclier Angles Morts (Survie)
#define SHIELD_MAX      100.0f
#define FORCE_SHIELD    400.0f

#define MIN_CLUSTER_PTS 5
#define CLUSTER_TOLERANCE 75.0f

#define BORDER_MARGIN 50.0f

extern float pf_goal_tolerance;
extern float pf_max_speed;
extern float pf_max_rotation;
extern float pf_attractive_gain;

extern float d_min;
extern float d_max;
extern float force_max;
extern float lateral_gain;
extern float shield_max;
extern float force_shield;  



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

void Path_GetRepulsionVector(const LD19DataPointHandler* scan, float motion_angle_rad, RobotPose current_pose, float *rep_vx, float *rep_vy);

void init_pathfinding_parameters(void);
void Set_pathfinding_parameters(float goal_tolerance, float max_speed, float max_rotation, float attractive_gain,
                                float dmin, float dmax, float fmax, float lat_gain, float shld_max, float f_shield);
uint8_t Set_Pathfinding_parameters_Cmd(void);
uint8_t Get_Pathfinding_parameters_Cmd(void);

#endif // PATHFINDING_H