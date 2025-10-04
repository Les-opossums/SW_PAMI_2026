#ifndef __ASSERV_H_
#define __ASSERV_H_


// mode de l'asservissement
#define ASSERV_MODE_OFF 0
#define ASSERV_MODE_FREE 1
#define ASSERV_MODE_BREAK 2
#define ASSERV_MODE_POS 10
#define ASSERV_MODE_SPEED 30
#define ASSERV_MODE_ABSOLUTE_SPEED 31

#define DIST_TOL 0.005 // 5mm
#define ANGLE_TOL 0.01745 // 1 deg


extern int motion_done;

extern int asserv_mode;

extern float blocked_time;

extern float current_stop_distance;
extern float default_stop_distance;

extern int emergency_break_requested;

/******************************    Fonctions    *******************************/

// initialiser le mode et les differents asservissements
void asserv_init(void);


// consignes de deplacements du robot
void motion_block(void) ;
void motion_free(void) ;
void motion_pos(Position pos);
void motion_off(void);

void motion_speed(Speed speed);
void motion_absolute_speed(Speed speed);



// effectue un pas d'asservissement
void motion_step(void);

// fonctions de calcul des commandes en fonction du mode
void asserv_off_step(void);
void asserv_free_step(void);

void pos_asserv_step(void);


void speed_asserv_step(void);
void speed_asserv_break_step(void);
void absolute_speed_asserv_step(void);

void asserv_check_blocked(float period);

// indique si l'asservissement en cours a termine
int Get_asserv_done();

float radial_speed_calculation(float distance);
float angular_speed_calculation(float angle);

#endif // _ASSERV_H_
