#include "comms.h"

static uint64_t mid_counter = 0;

double consiglio(double dist)
{
    double p1 = -7.44e-8;
    double p2 = -9.286e-5;
    double p3 = 0.9101;
    return p1 * (dist * dist) + p2 * dist + p3;
}

double esat(double dist)
{
    double r;
    if (dist < 50)
    {
        r = 1.0;
    }
    double p1 = -2.1e-09;
    double p2 = 5.034e-06;
    double p3 = -0.003541;
    double p4 = 1.138;

    r = p1 * dist * dist * dist + p2 * dist * dist + p3 * dist + p4;

    return r;
}
