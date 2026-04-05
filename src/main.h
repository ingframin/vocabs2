
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
#include <cstring>
time_t t;

#include "simulation_context.h"  // For SimulationContext
#include "file_system.h"

void displayHelp() {
    printf("Vocabs2 - Velocity Obstacle for Drones Simulator\n");
    printf("Usage: vocabs2 [OPTIONS]\n");
    printf("\nOptions:\n");
    printf("  -h, --help          Display this help message\n");
    printf("  -c, --config FILE   Use configuration from FILE (overrides all other parameters)\n");
    printf("  -p, --prob CHAR     Set system type: A=No loss, E=Wi-Fi, C=ADS-B (default: A)\n");
    printf("  -e, --error VALUE    Set positional error in meters (default: 0.0)\n");
    printf("  -l, --loss VALUE     Set loss probability (0.0 to 1.0, default: 1000.0)\n");
    printf("  -s, --speed VALUE    Set drone speed in m/s (default: 20.0)\n");
    printf("  -x, --ptx VALUE      Set transmit power (0.0 to 1.0, default: 0.5)\n");
    printf("  -r, --prx VALUE      Set receive power (0.0 to 1.0, default: 0.5)\n");
    printf("  -i, --pint VALUE     Set interference probability (0.0 to 1.0, default: 0.0)\n");
    printf("\nIf no config file is specified, default config.ini will be used.\n");
    printf("Command line parameters override config file settings.\n");
}

void parseArguments(int argc, char *argv[]) {
    char* config_file = NULL;
    
    // Check for help flag
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            displayHelp();
            exit(0);
        }
    }
    
    // Parse arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--config") == 0) {
            if (i + 1 < argc) {
                config_file = argv[++i];
            }
        } else if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--prob") == 0) {
            if (i + 1 < argc) {
                sim_context.prob = argv[++i][0];
            }
        } else if (strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "--error") == 0) {
            if (i + 1 < argc) {
                sim_context.error = atof(argv[++i]);
            }
        } else if (strcmp(argv[i], "-l") == 0 || strcmp(argv[i], "--loss") == 0) {
            if (i + 1 < argc) {
                sim_context.l = 1000.0 * atof(argv[++i]);
            }
        } else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--speed") == 0) {
            if (i + 1 < argc) {
                sim_context.speed = atof(argv[++i]);
                if (sim_context.speed > 100.0)
                {
                    sim_context.dt = 1e-4;
                }
            }
        } else if (strcmp(argv[i], "-x") == 0 || strcmp(argv[i], "--ptx") == 0) {
            if (i + 1 < argc) {
                sim_context.Ptx = atof(argv[++i]);
            }
        } else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--prx") == 0) {
            if (i + 1 < argc) {
                sim_context.Prx = atof(argv[++i]);
            }
        } else if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--pint") == 0) {
            if (i + 1 < argc) {
                sim_context.Pint = atof(argv[++i]);
            }
        }
    }
    
    // If config file is specified, load it and override all settings
    if (config_file != NULL) {
        Config config = parse_config(config_file);
        
        sim_context.iterations = config.iterations;
        sim_context.dt = config.dt;
        sim_context.error = config.error;
        
        // Copy the rates array
        if (config.rates != NULL && config.num_rates > 0) {
            if (sim_context.rates != NULL) {
                free(sim_context.rates);
            }
            sim_context.rates = (double*)malloc(config.num_rates * sizeof(double));
            if (sim_context.rates != NULL) {
                memcpy(sim_context.rates, config.rates, config.num_rates * sizeof(double));
            }
            sim_context.len_rates = config.num_rates;
        }
        
        sim_context.num_threads = config.num_threads;
        sim_context.speed = config.speed;
        sim_context.prob = config.prob;
        sim_context.l = config.loss;
        sim_context.Ptx = config.Ptx;
        sim_context.Prx = config.Prx;
        sim_context.Pint = config.Pint;
        
        // Set system type based on prob
        switch (sim_context.prob)
        {
        case 'E':
            sim_context.sys = WI_FI;
            break;
        case 'C':
            sim_context.sys = ADS_B;
            break;
        default:
            sim_context.sys = NO_LOSS;
        }
        
        free_config(&config);
    }
    
    // Display current configuration
    printf("Error: %.3f\n", sim_context.error);
    switch (sim_context.prob)
    {
    case 'E':
        printf("Wi-Fi beacons\n");
        break;
    case 'C':
        printf("ADS-B\n");
        break;
    default:
        printf("No loss\n");
    }
    printf("Loss: %.3f\n", sim_context.l);
    printf("Speed: %.3f\n", sim_context.speed);
}

/*Maybe I should add some error checking? :-/*/
#endif