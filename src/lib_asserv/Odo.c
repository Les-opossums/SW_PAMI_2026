#include "lib_asserv.h"


/******************************    Variables    *******************************/
float robot_wheel_distance;

Speed speed_robot;

Position position_robot; // Position odometrique
Position position_robot_cumul; // cumul des steps pour le calcul de la vitesse

Acceleration acceleration_robot;

int32_t step_1, step_2, step_3 = 0; // position en step des roues
int32_t cumul_step_1, cumul_step_2, cumul_step_3 = 0; // cumul des steps pour le calcul de la vitesse

float speed_1, speed_2, speed_3 = 0; // vitesses des roues en m/s

/******************************    Fonctions    *******************************/

// initialiser l'odometrie
void odo_init(void) {
    odo_set_spacing(DEFAULT_ODO_SPACING);

    position_robot.x = 0;
    position_robot.y = 0;
    position_robot.t = 0;

    speed_robot.vx = 0;
    speed_robot.vy = 0;
    speed_robot.vt = 0;

    acceleration_robot.ax = 0;
    acceleration_robot.ay = 0;
    acceleration_robot.at = 0;
}

// assigner une valeur e l'ecart entre les roues d'odometrie
void odo_set_spacing(float param_spacing) {
    robot_wheel_distance = param_spacing;
}

void odo_position_step(int32_t pos1, int32_t pos2, int32_t pos3) {
    //calculs des pos intermédiaires des roues 
    const float step_to_m = (DEFAULT_SIZE_WHEEL * M_PI) / (200.0f * 16.0f);  
    float delta_pos1 = (float)((int32_t)(pos1 - step_1)) * step_to_m;
    float delta_pos2 = (float)((int32_t)(pos2 - step_2)) * step_to_m;
    float delta_pos3 = (float)((int32_t)(pos3 - step_3)) * step_to_m;

    // calculs du déplacement depuis le dernier step
    float dx = (2.0f/3.0f) * (delta_pos1) - (1.0f/3.0f) * (delta_pos2 + delta_pos3);
    float dy = (sqrtf(3.0f) / 3.0f) * (delta_pos2 - delta_pos3); // translation avant (X robot)
    float dt = -(delta_pos1 + delta_pos2 + delta_pos3) / (3.0f * robot_wheel_distance);

    // cumul des steps
    position_robot_cumul.x += dx;
    position_robot_cumul.y += dy;
    position_robot_cumul.t += dt;

    // maj des positions des roues
    step_1 = pos1;
    step_2 = pos2;
    step_3 = pos3;
    
    // maj de la position odometrique pur
    float cos_t = cosf(position_robot.t);
    float sin_t = sinf(position_robot.t);

    float dx_global = dx * cos_t - dy * sin_t;
    float dy_global = dx * sin_t + dy * cos_t;
    
    position_robot.x += dx_global;
    position_robot.y += dy_global;
    position_robot.t = principal_angle(position_robot.t + dt);
}

void odo_speed_step(float period) {
    // sauvegarde des vitesses precedentes
    float old_vx = speed_robot.vx;
    float old_vy = speed_robot.vy;
    float old_vt = speed_robot.vt;

    // calcul des vitesses dans le repere robot
    speed_robot.vx = position_robot_cumul.x / period;
    speed_robot.vy = position_robot_cumul.y / period;
    speed_robot.vt = position_robot_cumul.t / period;

    // reset les cumuls
    position_robot_cumul.x = 0;
    position_robot_cumul.y = 0;
    position_robot_cumul.t = 0;

    // calcul des accelerations
    acceleration_robot.ax = (speed_robot.vx - old_vx);
    acceleration_robot.ay = (speed_robot.vy - old_vy);
    acceleration_robot.at = (speed_robot.vt - old_vt);
}

void set_position(Position pos) {
    position_robot = pos;
}

void set_position_x(float x) {
    position_robot.x = x;
}

void set_position_y(float y) {
    position_robot.y = y;
}

void set_position_t(float t) {
    position_robot.t = t;
}


Position get_position(void) {
    return position_robot;
}

Speed get_speed(void) {
    return speed_robot;
}

Acceleration get_acceleration(void) {
    return acceleration_robot;
}
