
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

typedef enum{
  OOK,
  FSK,
  GFSK,
  BPSK,
  QPSK,
  QAM
} ModulationFamily;

typedef struct modulation{
  double bandwidth;
  uint32_t bits_per_symbol;
  uint32_t symbol_time;//Symbol t ime in ns
  ModulationFamily family;
}Modulation;

typedef struct channel{
    double center_frequency;
    double bandwidth;
    double noise_power;
}Channel;


//Compute receive probability
//dist is the distance
double consiglio(double dist);
double esat(double dist);

//Compute the probability of packet errors
//Assuming power in dB, frequency in MHz
double COM_compute_Pe(const Channel* chn, double dist, double ptx, double symbol_rate, long packet_length, double(*recv_pwr)(double,double,double));
double COM_log_distance_Prx(double frequency, double dist, double Ptx);
bool COM_broadcast(vec2 d1, vec2 d2, RFsystem rf, double loss);
bool COM_broadcast_Pint(double Ptx, double Prx, double Pint);
#endif
