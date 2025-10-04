#ifndef __TOOLS_ASSERV_H_
#define __TOOLS_ASSERV_H_

// constante pi
#define PI 3.14159265359
#define TWO_PI 6.28318530718

// Fonctions generiques pouvant servir a plusieurs endroits

// Renvoyer une valeur comprise entre inf et sup
float limit_float(float valeur, float inf, float sup);


// angle principal
float principal_angle(float angle);

float maximum3(float a, float b, float c);
#endif


