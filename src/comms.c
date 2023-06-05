
/* 
Vocabs2 - velocity obstacle for drones simulator
Copyright (C) 2023  Franco Minucci

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "comms.h"
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif
#define C 299792458
#include <stdlib.h>

bool COM_broadcast_Pint(double Ptx, double Prx, double Pint){
    int p = rand() % 1000;
    double pcheck = 1000*Ptx*Prx*(1-Pint);
    return p < pcheck;
}

bool COM_broadcast(vec2 d1, vec2 d2, RFsystem sys, double loss)
{ // Change to wi-fi or ads-b functions
    int p = rand() % 1000;
    double lim;
    switch (sys)
    {
    case WI_FI:
        lim = loss * esat(v2_distance(d1, d2));
        break;
    case ADS_B:
        lim = loss * consiglio(v2_distance(d1, d2));
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

//Frequency in MHz
double COM_log_distance_Prx(double frequency, double dist, double Ptx){
  return Ptx - 20*log10(dist) - 20*log10(frequency*1e6) - 20*log10(4*M_PI/C);
}


double COM_compute_Pe(const Channel* chn, double dist, double ptx, double symbol_rate, long packet_length){
    //This should be a separate function and part of the channel model
    double prx = COM_log_distance_Prx(chn->center_frequency,dist,ptx);
    //To be replaced with SINR
    //Interference: Compute received power from other drones + other sources
    double snr = pow(10, (prx-chn->noise_power)/10);
    /*  Assuming 1 bit per symbol and binary modulation
    *   coherent detection
    *   Additive White Gaussian Noise
    */
    double eb_n0 = snr*chn->bandwidth/symbol_rate;
    double ber = 0.5*erfc(sqrt(eb_n0));//Depends on modulation + coding
    double per = 1-pow((1-ber),packet_length);
    
    return per;
    
}
