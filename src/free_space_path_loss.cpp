#include "pathloss.h"
#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif

FreeSpacePL::FreeSpacePL(double frequency, double bandwidth, double gain_tx, double gain_rx, double noise_level){
    this->frequency = frequency*1e6;
    this->bandwidth = bandwidth;
    this->gain_tx = gain_tx;
    this->gain_rx = gain_rx;
    base_PL = 20*log10(frequency)+20*log10(4*M_PI/C);
}

double FreeSpacePL::loss_dB(double distance){
    return 20*log10(distance)+base_PL;
}

double receivedPower(double distance, double transmitPower){
    return transmitPower+gain_tx+gain_rx-loss_dB(distance);
}

double SNR(double distance, double transmitPower, double noiseInterferenceLevel){
    return receivedPower(distance,transmitPower)-noiseInterferenceLevel;
}