#ifndef COMMS_H
#define COMMS_H
#include <stdint.h>
#include "vec2.h"

typedef struct
{

} Radio;

//Compute receive probability
double consiglio(double dist);
double esat(double dist);

#endif