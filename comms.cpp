#include "comms.h"
#include "rand_numbers.h"
 

bool COM_broadcast(vec2 d1, vec2 d2, RFsystem sys, double loss)
{ // Change to wi-fi or ads-b functions
    static RandomNum rng {1000.0};
    auto p = rng.getDouble();
    double lim;
    switch (sys)
    {
    case WI_FI:
        lim = loss * esat(d1.distance(d2));
        break;
    case ADS_B:
        lim = loss * consiglio(d1.distance(d2));
        break;
    default:
        lim = loss;
    }

    //If broadcast return true
    return p < lim;
}

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
        return 1.0;
    }
    double p1 = -2.1e-09;
    double p2 = 5.034e-06;
    double p3 = -0.003541;
    double p4 = 1.138;

    r = p1 * dist * dist * dist + p2 * dist * dist + p3 * dist + p4;

    return r;
}
