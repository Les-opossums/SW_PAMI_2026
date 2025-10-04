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


/**************************** PID  *****************************/
typedef struct {
    float kp;
    float ki;
    float kd;
} PID_coef;

typedef struct {
    float err;
    float err_int;
    float err_der;
} PID_err;

typedef struct {
    PID_coef coef;
    PID_err err1;
    PID_err err2;
    PID_err err3;
} PID_speed;

/**************************** Motor Command  ****************************/

typedef struct {
    float command1; // en %
    float command2; // en %
    float command3; // en %
} MOTOR_Command;

extern MOTOR_Command Consigne;
extern MOTOR_Command Wanted_Forced_Consigne;
extern MOTOR_Command old_Consigne;

#endif