#include "PAMI_2026.h"
#include "lib_asserv/Lib_Asserv.h"

uint32_t Last_Timer_print_pos = 0;

uint8_t auto_printpos_en = 0;
uint16_t auto_printpos_delay = 100;

uint8_t Debug_Timing = 0;

uint16_t Asserv_Full_Count = 0;

uint8_t Channel_Motor1 = 0;
uint8_t Channel_Motor2 = 1;
uint8_t Channel_Motor3 = 2;

float wheel_speed1 = 0;
float wheel_speed2 = 0;
float wheel_speed3 = 0;

uint8_t stop = 0;

int Last_Timer_Asserv = 0;
int Asserv_State = 0;
int Asserv_Odo_Count = 0;

MOTOR_Command Consigne;
MOTOR_Command Wanted_Forced_Consigne;
MOTOR_Command old_Consigne;

float dx, dy, dt = 0;

void Init_Asserv(void) {
    Consigne.command1 = 0;
    Consigne.command2 = 0;
    Consigne.command3 = 0;

    Wanted_Forced_Consigne.command1 = 0;
    Wanted_Forced_Consigne.command2 = 0;
    Wanted_Forced_Consigne.command3 = 0;

    old_Consigne.command1 = 0;
    old_Consigne.command2 = 0;
    old_Consigne.command3 = 0;

    asserv_init();
}

void Asserv_Loop(void)
{
	if (Asserv_State == 0) {
        if ((Timer_ms1 - Last_Timer_Asserv) > ODO_EVERY_MS) {
            Last_Timer_Asserv += ODO_EVERY_MS;
            Asserv_State = 2;
        }

    } else if (Asserv_State == 2) {
        //-----------------------------------
        // pos step
        //-----------------------------------
        odo_position_step(motors[0].current_position, 
                            motors[1].current_position, 
                            motors[2].current_position);
        Asserv_Odo_Count ++;

        if (Asserv_Odo_Count >= ASSERV_EVERY){
            Asserv_Odo_Count = 0;
            Asserv_State ++;
        } else {
            Asserv_State = 0;
        }

    } else if (Asserv_State == 3) {
        //-----------------------------------
        // speed step
        //-----------------------------------
        odo_speed_step(ASSERV_EVERY*ODO_EVERY_MS*0.001f);
        Asserv_State = 10;
    
    } else if (Asserv_State == 10) {
        //-----------------------------------
        // motion step
        //-----------------------------------
        motion_step();
        Asserv_State = 20;

    } else if (Asserv_State == 20) {
        //-----------------------------------
        // spped constrain
        //-----------------------------------
        constrain_speed_order();
        Asserv_State = 21;

    } else if (Asserv_State == 21) {
        //-----------------------------------
        // acceleration constrain
        //-----------------------------------
        constrain_acceleration_order(ASSERV_EVERY*ODO_EVERY_MS*0.001f);
        Asserv_State = 30;

    } else if (Asserv_State == 30) {
        //-----------------------------------
        // consigne
        //-----------------------------------
        Asserv_PWM_calculator(&Consigne);
        Asserv_State = 40;


    } else if (Asserv_State == 40) {
        Move_Stepper(Consigne.command1, Consigne.command2, Consigne.command3);
        
        Asserv_State = 50;

    } else if (Asserv_State == 50) {
        Move_Stepper(Consigne.command1, Consigne.command2, Consigne.command3);
        Asserv_State = 60;

    } else if (Asserv_State == 60) {
        if (auto_printpos_en && ((Timer_ms1 - Last_Timer_print_pos) > auto_printpos_delay)) {
            float speed_linear = sqrtf(speed_robot.vx*speed_robot.vx + speed_robot.vy*speed_robot.vy);
            float speed_direction = atan2f(speed_robot.vy, speed_robot.vx);
            printf("ROBOTDATA %0.4f %0.4f %0.4f %0.2f %0.2f %0.2f\n", position_robot.x, position_robot.y, position_robot.t, speed_linear, speed_direction, speed_robot.vt);
            Last_Timer_print_pos += auto_printpos_delay;
        }
        Asserv_State = 0;
        
    } else {
        Asserv_State = 0;
    }
}

uint8_t Activate_Position_Sending_Func (void) {
    uint32_t state;
    if (Get_Param_u32(&state)) return PARAM_ERROR_CODE;
    auto_printpos_en = (state != 0);
    Last_Timer_print_pos = Timer_ms1;
    
    uint32_t Delay;
    if (!Get_Param_u32(&Delay)) {   // s'il y a un 2eme param, on s'en sert comme delai entre les prints (en ms, of course)
        auto_printpos_delay = Delay;
    }
    return 0;
}


uint8_t Set_Odo_Spacing_Cmd(void){
    float valf;
    if (Get_Param_Float(&valf))
        return 1;
    odo_set_spacing(valf);
    return 0;
}