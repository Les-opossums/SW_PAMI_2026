#ifndef __ODO_H_
#define __ODO_H_

extern float robot_wheel_distance;

/******************************    Fonctions    *******************************/

// initialiser l'odometrie
void odo_init(void);

// assigner une valeur a l'ecart entre les roues d'odometrie
void odo_set_spacing(float param_spacing);

// maj de la position du robot
void odo_position_step(int32_t pos1, int32_t pos2, int32_t pos3);

// maj de la vitesse/acceleration  du robot
void odo_speed_step(float period);

// connaitre l'etat du robot
Position get_position(void);
Speed get_speed(void);
Acceleration get_acceleration(void);


// assigner des valeurs a la position (x, y et theta)
void set_position(Position pos);
void set_position_x(float x);
void set_position_y(float y);
void set_position_t(float t);

#endif // _ODO_H_
