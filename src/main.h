
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

#ifndef MAIN_H
#define MAIN_H
#ifdef __unix
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),(mode)))==NULL
#endif
time_t t;

#include "simulation_context.h"  // For SimulationContext
#include "file_system.h"

void parseArguments(int argc, char *argv[]){
  if (argc > 1)
  {
    sim_context.prob = argv[1][0];
  }
  if (argc > 2)
  {
    sim_context.error = atof(argv[2]);
  }
  if (argc > 3)
  {
    sim_context.l = 1000.0 * atof(argv[3]);
  }
  if (argc > 4)
  {
    sim_context.speed = atof(argv[4]);
    if (sim_context.speed > 100.0)
    {
      sim_context.dt = 1e-4;
    }
  }
  printf("Error: %.3f\n", sim_context.error);
  switch (sim_context.prob)
  {
  case 'E':
    printf("Wi-Fi beacons\n");
    sim_context.sys = WI_FI;
    break;
  case 'C':
    printf("ADS-B\n");
    sim_context.sys = ADS_B;
    break;
  default:
    sim_context.sys = NO_LOSS;
    printf("No loss\n");
  }
  printf("Loss: %.3f\n", sim_context.l);
  printf("Speed: %.3f\n", sim_context.speed);

}

/*Maybe I should add some error checking? :-/*/
#endif