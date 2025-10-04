#ifndef __SPEED_CONSTRAINER_H_
#define __SPEED_CONSTRAINER_H_

extern Acceleration Accel_Max;

extern Speed speed_order;
extern Speed speed_order_constrained;

extern float Speed_Order_1, Speed_Order_2, Speed_Order_3;

extern float robot_v_max;
extern float robot_vt_max;

extern float robot_a_max;
extern float robot_at_max;



void speed_constrainer_init(void);
void acceleration_constrainer_init(void);

// contraint la consigne de vitesse selon les caracteristiques du robot
void constrain_speed_order(void);
void constrain_acceleration_order(float period);

void set_Constraint_vitesse_xy_max(float v_max);
void set_Constraint_vt_max(float vt_max);

void set_Constraint_a_xy_max(float a_max);
void set_Constraint_at_max(float at_max);

#endif // _ASSERV_H_
