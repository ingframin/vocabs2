
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

#ifndef SIMULATION_H
#define SIMULATION_H
#include "drone.h"
#include "vec2.h"

//This structure contains the status of the Simulation
typedef struct config{
    Drone* drones;//This is the base of a list
    uint64_t num_drones;
    vec2* points;
    uint64_t num_points;

} Configuration;

Configuration* Conf_newConfiguration(const char* configfile);
void Conf_freeConfiguration(Configuration* conf);
void Sim_run(const Configuration* conf);

#endif
