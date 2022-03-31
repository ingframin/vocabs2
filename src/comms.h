#ifndef COMMS_H
#define COMMS_H
#include <stdint.h>
#include <stdbool.h>
#include "vec2.h"

typedef enum
{
    NO_LOSS,
    WI_FI,
    ADS_B
} RFsystem;

//Compute receive probability
//dist is the distance
double consiglio(double dist);
double esat(double dist);
//Assuming power in dB, frequency in MHz
double interference(double dist, double ptx, double noise_power, double frequency, double bandwidth, double symbol_rate, long packet_length);
bool COM_broadcast(vec2 d1, vec2 d2, RFsystem rf, double loss);

#endif