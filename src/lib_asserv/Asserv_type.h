#ifndef ASSERV_TYPE_H
#define ASSERV_TYPE_H

/*****************************    Odometrie    *******************************/

// Position absolue du robot (x, y, et theta)
typedef struct {
    float x; // en metre
    float y; // en metre
    float t; // en radian
} Position;

extern Position position_robot;

// Vitesse et vitesse angulaire du robot
typedef struct {
    float vx; // en m/s
    float vy; // en m/s
    float vt; // en rad/s
} Speed;

extern Position Wanted_Pos;
extern Speed Wanted_Speed;
extern Speed speed_robot;
extern Speed speed_robot_odom;

// acceleration du robot (dv/dt,  d2theta/dt2   et   v*(dtheta/dt))
typedef struct {
    float ax; // en m/s2
    float ay; // en m/s2
    float at; // en rad/s2
} Acceleration;

extern Acceleration acceleration_robot;

/**************************** Motor Command  ****************************/

typedef struct {
    int32_t command1;
    int32_t command2;
    int32_t command3;
} MOTOR_Command;

extern MOTOR_Command Consigne;
extern MOTOR_Command Wanted_Forced_Consigne;
extern MOTOR_Command old_Consigne;

#endif