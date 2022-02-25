#ifndef FS_PATHLOSS_H
#define FS_PATHLOSS_H
#include "pathloss.h"

class FreeSpacePL: public PathLoss
{
public:
    FreeSpacePL(double frequency, double bandwidth, double gain_tx, double gain_rx);
    double loss_dB(double distance);
    double receivedPower(double distance, double transmitPower);
    double SNR(double distance, double transmitPower, double noiseInterferenceLevel);
    
private:
    double gain_tx;
    double gain_rx;
    double base_PL;
}

#endif