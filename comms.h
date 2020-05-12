#ifndef COMMS_H
#define COMMS_H
#include <stdint.h>
#include "vec2.h"

typedef struct
{
    uint64_t MID;
    vec2 position;
    vec2 speed;
    double size;

} Message;

//if a message is invalid its ID is -1

Message broadcast(double x, double y, double vx, double vy, double size);
uint32_t listen(Message *buffer, double (*rx_prob)(double));

//Compute receive probability
double consiglio(double dist);
double esat(double dist);

#endif