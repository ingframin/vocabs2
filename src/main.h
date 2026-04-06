
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

#include <string>
#include <iostream>
#include <iomanip>
time_t t;

#include "simulation_context.h"  // For SimulationContext
#include "file_system.h"

void displayHelp() {
    std::cout << "Vocabs2 - Velocity Obstacle for Drones Simulator\n";
    std::cout << "Usage: vocabs2 [OPTIONS]\n";
    std::cout << "\nOptions:\n";
    std::cout << "  -h, --help          Display this help message\n";
    std::cout << "  -c, --config FILE   Use configuration from FILE (overrides all other parameters)\n";
    std::cout << "  -o, --output FILE   Set output filename for results\n";
    std::cout << "  -p, --prob CHAR     Set system type: A=No loss, E=Wi-Fi, C=ADS-B (default: A)\n";
    std::cout << "  -e, --error VALUE    Set positional error in meters (default: 0.0)\n";
    std::cout << "  -l, --loss VALUE     Set loss probability (0.0 to 1.0, default: 1000.0)\n";
    std::cout << "  -s, --speed VALUE    Set drone speed in m/s (default: 20.0)\n";
    std::cout << "  -x, --ptx VALUE      Set transmit power (0.0 to 1.0, default: 0.5)\n";
    std::cout << "  -r, --prx VALUE      Set receive power (0.0 to 1.0, default: 0.5)\n";
    std::cout << "  -i, --pint VALUE     Set interference probability (0.0 to 1.0, default: 0.0)\n";
    std::cout << "\nIf no config file is specified, default config.ini will be used.\n";
    std::cout << "Command line parameters override config file settings.\n";
}

void parseArguments(int argc, char *argv[], std::string& output_filename) {
    std::string config_file = "";
    
    // Check for help flag
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            displayHelp();
            exit(0);
        }
    }
    
    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-o" || arg == "--output") {
            if (i + 1 < argc) {
                output_filename = argv[++i];
            }
        } else if (arg == "-c" || arg == "--config") {
            if (i + 1 < argc) {
                config_file = argv[++i];
            }
        } else if (arg == "-p" || arg == "--prob") {
            if (i + 1 < argc) {
                sim_context.prob = argv[++i][0];
            }
        } else if (arg == "-e" || arg == "--error") {
            if (i + 1 < argc) {
                sim_context.error = atof(argv[++i]);
            }
        } else if (arg == "-l" || arg == "--loss") {
            if (i + 1 < argc) {
                sim_context.l = 1000.0 * atof(argv[++i]);
            }
        } else if (arg == "-s" || arg == "--speed") {
            if (i + 1 < argc) {
                sim_context.speed = atof(argv[++i]);
                if (sim_context.speed > 100.0)
                {
                    sim_context.dt = 1e-4;
                }
            }
        } else if (arg == "-x" || arg == "--ptx") {
            if (i + 1 < argc) {
                sim_context.Ptx = atof(argv[++i]);
            }
        } else if (arg == "-r" || arg == "--prx") {
            if (i + 1 < argc) {
                sim_context.Prx = atof(argv[++i]);
            }
        } else if (arg == "-i" || arg == "--pint") {
            if (i + 1 < argc) {
                sim_context.Pint = atof(argv[++i]);
            }
        }
    }
    
    // If config file is specified, load it and override all settings
    if (!config_file.empty()) {
        Config config = parse_config(config_file);
        
        sim_context.iterations = config.iterations;
        sim_context.dt = config.dt;
        sim_context.error = config.error;
        
        // Copy the rates array
        if (config.rates != NULL && config.num_rates > 0) {
            // Vector handles its own memory management
            sim_context.rates.assign(config.rates, config.rates + config.num_rates);
        }
        
        sim_context.num_threads = config.num_threads;
        sim_context.speed = config.speed;
        sim_context.prob = config.prob;
        sim_context.l = config.loss;
        sim_context.Ptx = config.Ptx;
        sim_context.Prx = config.Prx;
        sim_context.Pint = config.Pint;
        
        // Copy filename if not already set via command line
        if (output_filename.empty() && !config.filename.empty()) {
            sim_context.filename = config.filename;
            output_filename = sim_context.filename;
        }
        
        // Set system type based on prob
        switch (sim_context.prob)
        {
        case 'E':
            sim_context.sys = RFsystem::WI_FI;
            break;
        case 'C':
            sim_context.sys = RFsystem::ADS_B;
            break;
        default:
            sim_context.sys = RFsystem::NO_LOSS;
        }
        
        free_config(&config);
    }
    
    // Display current configuration
    std::cout << "Error: " << std::fixed << std::setprecision(3) << sim_context.error << "\n";
    switch (sim_context.prob)
    {
    case 'E':
        std::cout << "Wi-Fi beacons\n";
        break;
    case 'C':
        std::cout << "ADS-B\n";
        break;
    default:
        std::cout << "No loss\n";
    }
    std::cout << "Loss: " << std::fixed << std::setprecision(3) << sim_context.l << "\n";
    std::cout << "Speed: " << std::fixed << std::setprecision(3) << sim_context.speed << "\n";
}

/*Maybe I should add some error checking? :-/*/
#endif