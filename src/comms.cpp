
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
#include <stdlib.h>
#include <float.h>

// Helper function for input validation
static bool is_valid_probability(double p) {
    return p >= 0.0 && p <= 1.0 && !isnan(p) && !isinf(p);
}

// Helper function to clamp probability values
static double clamp_probability(double p) {
    if (p < 0.0) return 0.0;
    if (p > 1.0) return 1.0;
    if (isnan(p)) return 0.0;
    if (isinf(p)) return p > 0 ? 1.0 : 0.0;
    return p;
}

bool COM_broadcast_Pint(double Ptx, double Prx, double Pint){
    // Validate inputs
    if (!is_valid_probability(Ptx) || !is_valid_probability(Prx) || !is_valid_probability(Pint)) {
        // Clamp invalid values to safe ranges
        Ptx = clamp_probability(Ptx);
        Prx = clamp_probability(Prx);
        Pint = clamp_probability(Pint);
    }
    
    int p = rand() % 1000;
    double pcheck = PROBABILITY_SCALE * Ptx * Prx * (1 - Pint);
    return p < pcheck;
}

bool COM_broadcast(vec2 d1, vec2 d2, RFsystem sys, double loss)
{ // Change to wi-fi or ads-b functions
    // Validate loss parameter
    if (!is_valid_probability(loss)) {
        loss = clamp_probability(loss);
    }
    
    int p = rand() % 1000;
    double lim;
    double distance = d1.distanceTo(d2);
    
    // Handle invalid distance (shouldn't happen with valid vec2, but be safe)
    if (distance < 0) distance = 0;
    
    switch (sys)
    {
    case WI_FI:
        lim = loss * esat(distance);
        break;
    case ADS_B:
        lim = loss * consiglio(distance);
        break;
        
    default:
        lim = loss;
    }

    // Clamp lim to valid probability range
    if (lim < 0) lim = 0;
    if (lim > PROBABILITY_SCALE) lim = PROBABILITY_SCALE;

    //If broadcast return true
    return p < lim;
}
double consiglio(double dist)
{
    // Validate input
    if (isnan(dist) || isinf(dist)) {
        return 0.0;
    }
    if (dist < 0) {
        dist = 0;
    }
    // For very large distances, limit to asymptotic behavior
    if (dist > 1e6) {
        // Linear term dominates for large dist
        return -9.286e-5 * dist + 0.9101;
    }
    
    double p1 = -7.44e-8;
    double p2 = -9.286e-5;
    double p3 = 0.9101;
    double result = p1 * (dist * dist) + p2 * dist + p3;
    
    // Clamp result to valid probability range
    if (result < 0) return 0.0;
    if (result > 1.0) return 1.0;
    return result;
}

double esat(double dist)
{
    // Validate input
    if (isnan(dist) || isinf(dist)) {
        return 1.0; // Default to maximum probability for invalid input
    }
    if (dist < 0) {
        dist = 0;
    }
    
    // For distances < 50m, return maximum probability
    if (dist < 50)
    {
        return 1.0;
    }
    
    // For very large distances, limit to prevent numerical issues
    if (dist > 1e5) {
        // Higher order terms dominate - use quadratic approximation
        double p1 = -2.1e-09;
        double p2 = 5.034e-06;
        return p1 * dist * dist * dist + p2 * dist * dist;
    }
    
    double p1 = -2.1e-09;
    double p2 = 5.034e-06;
    double p3 = -0.003541;
    double p4 = 1.138;

    double r = p1 * dist * dist * dist + p2 * dist * dist + p3 * dist + p4;
    
    // Clamp result to valid probability range
    if (r < 0) return 0.0;
    if (r > 1.0) return 1.0;
    return r;
}

//Frequency in MHz
double COM_log_distance_Prx(double frequency, double dist, double Ptx){
    // Validate inputs
    if (frequency <= 0 || isnan(frequency) || isinf(frequency)) {
        frequency = 2.4; // Default to 2.4 GHz if invalid
    }
    if (dist <= 0 || isnan(dist) || isinf(dist)) {
        dist = MIN_DISTANCE; // Use minimum distance if invalid
    }
    if (Ptx < MIN_POWER || isnan(Ptx) || isinf(Ptx)) {
        Ptx = -30.0; // Default to -30 dBm if invalid
    }
    
    // Calculate using log-distance path loss model
    // Prx = Ptx - 20*log10(dist) - 20*log10(frequency) - 20*log10(4*pi/c)
    double result = Ptx - 20*log10(dist) - 20*log10(frequency*1e6) - 20*log10(4*M_PI/SPEED_OF_LIGHT);
    
    // Check for numerical issues
    if (isnan(result) || isinf(result)) {
        return MIN_POWER; // Return minimum power if calculation fails
    }
    
    return result;
}


double COM_compute_Pe(const Channel* chn, double dist, double ptx, double symbol_rate, long packet_length, double(*recv_pwr)(double,double,double)){
    // Validate inputs
    if (chn == NULL || recv_pwr == NULL) {
        return 1.0; // Maximum error rate for invalid channel or function
    }
    
    // Create local variables for channel parameters to handle invalid values
    double center_frequency = chn->center_frequency;
    double bandwidth = chn->bandwidth;
    double noise_power = chn->noise_power;
    
    if (dist <= 0 || isnan(dist) || isinf(dist)) {
        dist = MIN_DISTANCE;
    }
    
    if (ptx < MIN_POWER || isnan(ptx) || isinf(ptx)) {
        ptx = -30.0; // Default power
    }
    
    if (symbol_rate <= 0 || isnan(symbol_rate) || isinf(symbol_rate)) {
        symbol_rate = MIN_SYMBOL_RATE;
    }
    
    if (packet_length <= 0) {
        packet_length = 1; // Minimum packet length
    }
    
    if (center_frequency <= 0 || isnan(center_frequency) || isinf(center_frequency)) {
        center_frequency = 2400.0; // Default to 2.4 GHz
    }
    
    if (bandwidth <= 0 || isnan(bandwidth) || isinf(bandwidth)) {
        bandwidth = MIN_BANDWIDTH;
    }
    
    if (isnan(noise_power) || isinf(noise_power)) {
        noise_power = -100.0; // Default noise power
    }
    
    //This is a function passed as parameters: it takes the frequency, distance, and transmitted power.
    //It might be worth to pass some context as well. This is a limit of not having Classes.
    //Maybe I should consider moving to C++ or use another pattern?
    double prx = recv_pwr(center_frequency, dist, ptx);
    
    // Check for invalid received power
    if (isnan(prx) || isinf(prx)) {
        return 1.0; // Maximum error rate
    }
    
    //To be replaced with SINR
    //Interference: Compute received power from other drones + other sources
    double snr = pow(10, (prx - noise_power)/10);
    
    // Prevent division by zero in eb_n0 calculation
    if (symbol_rate < 1e-12) {
        return 1.0; // Maximum error rate if symbol rate is effectively zero
    }
    
    /*
    *   Maybe this should go into a structure?  
    *   Assuming 1 bit per symbol and binary modulation
    *   coherent detection
    *   Additive White Gaussian Noise
    */
    double eb_n0 = snr * bandwidth / symbol_rate;
    
    // Handle numerical issues in erfc
    if (eb_n0 < 0) eb_n0 = 0;
    if (eb_n0 > 100) eb_n0 = 100; // Limit to prevent erfc underflow
    
    double ber = 0.5*erfc(sqrt(eb_n0));//Depends on modulation + coding
    
    // Handle extreme BER values
    if (ber < 0) ber = 0;
    if (ber > 1.0) ber = 1.0;
    
    double per = 1 - pow((1 - ber), packet_length);
    
    // Clamp result to valid range
    if (per < 0) return 0.0;
    if (per > 1.0) return 1.0;
    
    return per;
    
}
