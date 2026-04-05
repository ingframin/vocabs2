/* 
Simulation Context for Vocabs2 - velocity obstacle for drones simulator
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

#ifndef SIMULATION_CONTEXT_H
#define SIMULATION_CONTEXT_H

#include <stdint.h>
#include <vector>
#include "comms.h"  // For RFsystem type

// Simulation context structure to hold all configuration and runtime state
struct SimulationContext{
    uint32_t iterations;
    double dt; //seconds
    double error; // positional error in meters
    std::vector<double> rates; // timings in ms
    int num_threads;
    double speed;
    int si;
    uint64_t rate;
    char prob;
    RFsystem sys;
    double l; // loss probability x1000
    // Communication parameters
    double Ptx;    // Transmit power (0.0 to 1.0)
    double Prx;    // Receive power (0.0 to 1.0)
    double Pint;   // Interference probability (0.0 to 1.0)
    // Output parameters
    std::string filename; // Output filename
};

// Global simulation context (defined in file_system.cpp, declared here)
extern SimulationContext sim_context;

#endif