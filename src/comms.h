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

typedef struct channel{
    double center_frequency;
    double bandwidth;
    double noise_power;
    double (*compute_loss)(double,double,double);
}Channel;

//Compute receive probability
//dist is the distance
double consiglio(double dist);
double esat(double dist);

//Compute the probability of packet errors
//Assuming power in dB, frequency in MHz
double COM_compute_Pe(const Channel* chn, double dist, double ptx, double symbol_rate, long packet_length);

bool COM_broadcast(vec2 d1, vec2 d2, RFsystem rf, double loss);

#endif