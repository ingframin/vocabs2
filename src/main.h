
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

// Configuration variables (will be loaded from config file)
uint32_t iterations;
double dt; //seconds
double error; // positional error in meters
double* rates; // timings in ms
int num_threads;
double speed;
int si = 0;
uint64_t rate = 1;
char prob;
RFsystem sys;
double l; // loss probability x1000
uint32_t len_rates;

#include "file_system.h"

void load_config() {
    Config config = parse_config("config.ini");
    
    iterations = config.iterations;
    dt = config.dt;
    error = config.error;
    rates = config.rates;
    len_rates = config.num_rates;
    num_threads = config.num_threads;
    speed = config.speed;
    prob = config.prob;
    l = config.loss;
    
    // Don't free config.rates here as we're using it directly
    // The rates array is now managed by the main program
}

void parseArguments(int argc, char *argv[]){
  if (argc > 1)
  {
    prob = argv[1][0];
  }
  if (argc > 2)
  {
    error = atof(argv[2]);
  }
  if (argc > 3)
  {
    l = 1000.0 * atof(argv[3]);
  }
  if (argc > 4)
  {
    speed = atof(argv[4]);
    if (speed > 100.0)
    {
      dt = 1e-4;
    }
  }
  printf("Error: %.3f\n", error);
  switch (prob)
  {
  case 'E':
    printf("Wi-Fi beacons\n");
    sys = WI_FI;
    break;
  case 'C':
    printf("ADS-B\n");
    sys = ADS_B;
    break;
  default:
    sys = NO_LOSS;
    printf("No loss\n");
  }
  printf("Loss: %.3f\n", l);
  printf("Speed: %.3f\n", speed);

}

/*Maybe I should add some error checking? :-/*/
void saveResults(const char* filename, double collisions[]){
  FILE *results;
  fopen_s(&results,filename, "a");
  fprintf(results, "Error: %.3f\n", error);
  fprintf(results, "Loss: %.3f\n", l);
  fprintf(results, "Speed: %.3f\n", speed);
  switch (prob)
  {
    case 'E':
      fprintf(results, "Wi-Fi beacons\n");
      break;
    case 'C':
      fprintf(results, "ADS-B\n");
      break;
    default:
      fprintf(results, "No loss\n");
  }

  for (uint32_t k = 0; k < len_rates; k++)
  {
    printf("%.3f\t%.6f\n", 1000.0/rates[k], collisions[k] / iterations);
    fprintf(results, "%.3f\t%.10f\n", 1000.0/rates[k], collisions[k] / iterations);
  }
  fclose(results);
}

#endif