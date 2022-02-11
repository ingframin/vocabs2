#ifndef COMMS_H
#define COMMS_H
#include "vec2.h"


typedef enum
{
    NO_LOSS,
    WI_FI,
    ADS_B
} RFsystem;
//Compute receive probability
double consiglio(double dist);
double esat(double dist);
bool COM_broadcast(vec2 d1, vec2 d2, RFsystem rf, double loss);
#endif