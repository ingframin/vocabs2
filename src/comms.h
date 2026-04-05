
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
#include <limits.h>
#include "math2d.h"

// Physical constants
#define SPEED_OF_LIGHT 299792458.0 // m/s
#define PROBABILITY_SCALE 1000.0    // Scale factor for probability calculations

// Minimum valid values for numerical stability
#define MIN_DISTANCE 1e-6         // Minimum distance (meters)
#define MIN_POWER -200.0          // Minimum power (dB)
#define MIN_BANDWIDTH 1e3         // Minimum bandwidth (Hz)
#define MIN_SYMBOL_RATE 1e3       // Minimum symbol rate (symbols/s)

enum class RFsystem {
    NO_LOSS,   /**< No loss model */
    WI_FI,     /**< Wi-Fi beacon model */
    ADS_B      /**< ADS-B model */
};

enum class ModulationFamily {
    OOK,       /**< On-Off Keying */
    FSK,       /**< Frequency Shift Keying */
    GFSK,      /**< Gaussian Frequency Shift Keying */
    BPSK,      /**< Binary Phase Shift Keying */
    QPSK,      /**< Quadrature Phase Shift Keying */
    QAM        /**< Quadrature Amplitude Modulation */
};

struct Modulation{
  double bandwidth;        /**< Bandwidth in Hz */
  uint32_t bits_per_symbol; /**< Bits per symbol */
  uint32_t symbol_time;    /**< Symbol time in ns */
  ModulationFamily family; /**< Modulation type */
};

struct Channel{
    double center_frequency; /**< Center frequency in MHz */
    double bandwidth;        /**< Bandwidth in Hz */
    double noise_power;      /**< Noise power in dB */
};

/**
 * @brief Compute reception probability using Consiglio model
 * @param dist Distance in meters (must be >= 0)
 * @return Probability (0.0 to 1.0)
 */
double consiglio(double dist);

/**
 * @brief Compute reception probability using ESAT model
 * @param dist Distance in meters (must be >= 0)
 * @return Probability (0.0 to 1.0)
 */
double esat(double dist);

/**
 * @brief Compute probability of packet errors
 * @param chn Channel parameters
 * @param dist Distance in meters (must be > 0)
 * @param ptx Transmit power in dB (must be > MIN_POWER)
 * @param symbol_rate Symbol rate in symbols/s (must be > 0)
 * @param packet_length Packet length in bits (must be > 0)
 * @param recv_pwr Function pointer to receive power calculation function
 * @return Packet error rate (0.0 to 1.0)
 */
double COM_compute_Pe(const Channel* chn, double dist, double ptx, double symbol_rate, long packet_length, double(*recv_pwr)(double,double,double));

/**
 * @brief Compute received power using log-distance path loss model
 * @param frequency Frequency in MHz (must be > 0)
 * @param dist Distance in meters (must be > 0)
 * @param Ptx Transmit power in dB (must be > MIN_POWER)
 * @return Received power in dB
 */
double COM_log_distance_Prx(double frequency, double dist, double Ptx);

/**
 * @brief Simulate broadcast with probability-based interference
 * @param d1 Position of first drone
 * @param d2 Position of second drone
 * @param rf RF system type
 * @param loss Loss probability factor (0.0 to 1.0)
 * @return True if broadcast successful, false otherwise
 */
bool COM_broadcast(vec2 d1, vec2 d2, RFsystem rf, double loss);

/**
 * @brief Simulate broadcast with explicit power levels and interference
 * @param Ptx Transmit power (0.0 to 1.0)
 * @param Prx Receive power (0.0 to 1.0)
 * @param Pint Interference probability (0.0 to 1.0)
 * @return True if broadcast successful, false otherwise
 */
bool COM_broadcast_Pint(double Ptx, double Prx, double Pint);

#endif
