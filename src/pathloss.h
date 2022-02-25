#ifndef PATHLOSS_H
#define PATHLOSS_H
#define C 299792458
/*
* Path Loss object interface:
* The idea is to have a common interface for different channel models which can be swapped as needed.
* The default can be Free Space path loss but more complex models can be used involving antenna patternd,
* ray tracing, fading, etc... 
* !!! IMPORTANT!!!
* - Gain, SNR and Power are in dB
* - Distances and wavelengths are in meters
* - Frequencies are in MHz
* - Speeds are in m/s
*/


class PathLoss{
public:
    virtual double loss_dB(double distance)=0;
    virtual double receivedPower(double distance, double transmitPower)=0;
    virtual double SNR(double distance, double transmitPower, double noiseInterferenceLevel)=0;


protected:
    double bandwidth
    double centre_freq;
    
    
};
#endif