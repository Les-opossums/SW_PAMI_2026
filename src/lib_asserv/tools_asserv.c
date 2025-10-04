
#include "lib_asserv.h"


// Renvoyer une valeur comprise entre inf et sup

float limit_float(float valeur, float inf, float sup) {
    if (valeur < inf) return inf;
    else if (valeur > sup) return sup;
    else return valeur;
}

// angle principal

float principal_angle(float angle) {
    return fmodf(angle + PI, 2 * PI) - PI;
}

float maximum3(float a, float b, float c) {
    if (a > b) {
        if (a > c) return a;
        else return c;
    } else {
        if (b > c) return b;
        else return c;
    }
}
