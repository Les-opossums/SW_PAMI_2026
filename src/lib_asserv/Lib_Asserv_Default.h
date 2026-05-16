#ifndef _LIB_ASSERV_DEFAULT_H_
#define _LIB_ASSERV_DEFAULT_H_

/*############################################################################*/
/*                                    Odo                                     */
/*############################################################################*/

// {tic/m, m/tic, entre roues}
#define DEFAULT_ODO_SPACING  0.045
#define DEFAULT_SIZE_WHEEL 0.050
#define DEFAULT_WHEEL_RADIUS DEFAULT_SIZE_WHEEL/2 
#define DEFAULT_STEPS_PER_REV 16 * 200 // 1/16 de microstep sur un moteur 200 pas (1.8° par pas)

/*############################################################################*/
/*                                  Motion                                    */
/*############################################################################*/
#ifndef PAMI_DUAL_WHEEL
#define PAMI_DUAL_WHEEL 0
#endif

#if PAMI_DUAL_WHEEL
    #define DEFAULT_CONSTRAINT_V_MAX 0.7f //0.8
    #define DEFAULT_CONSTRAINT_VT_MAX 1.0f

    #define DEFAULT_CONSTRAINT_A_MAX 0.5f //0.6
    #define DEFAULT_CONSTRAINT_AT_MAX 1.0f

    #define ASSERV_BLOCK_TIME_LIMIT 1   // 1s "blocké" avant de tout couper
#else
    #define DEFAULT_CONSTRAINT_V_MAX 0.3f //0.8
    #define DEFAULT_CONSTRAINT_VT_MAX 1.0f

    #define DEFAULT_CONSTRAINT_A_MAX 0.3f //0.6
    #define DEFAULT_CONSTRAINT_AT_MAX 1.0f

    #define ASSERV_BLOCK_TIME_LIMIT 1   // 1s "blocké" avant de tout couper
#endif

/*############################################################################*/
/*                                  Asserv                                    */
/*############################################################################*/

#define DEFAULT_STOP_DISTANCE 0.005 // +-5mm
#define DEFAULT_STOP_ANGLE 0.01745// +-1deg  // en radian

#define DEFAULT_SPEED_LIN_STOP 0.05 // 5cm/s
#define DEFAULT_SPEED_ROT_STOP 0.05 // 5rad/s

#endif // _LIB_ASSERV_DEFAULT_H_

