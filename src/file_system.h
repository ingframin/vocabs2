
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

#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdint.h>
#ifdef __unix
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),(mode)))==NULL
#define strcat_s(dest,size,source) strcat((dest),(source))
#endif

#include "simulation_context.h"  // For SimulationContext

typedef struct content{
    char* text;
    uint32_t size;
}Text;

// Modern C++ version that returns string instead of Text struct
std::string read_text_file_modern(const std::string& filename);
// Legacy versions for backward compatibility
Text read_text_file(const char* filename);
Text read_text_file_legacy(const std::string& filename);
void write_text_file(const std::string& filename, const std::string& content);
void write_text_file_modern(const std::string& filename, const std::string& content);
// Legacy versions for backward compatibility  
void write_text_file(const char* filename, const Text* text);
void write_text_file_legacy(const std::string& filename, const Text* text);

typedef struct config {
    uint32_t iterations;
    double dt;
    double error;
    double* rates;
    uint32_t num_rates;
    int num_threads;
    double speed;
    char prob;
    double loss;
    int num_drones; // Number of drones
    // Communication parameters
    double Ptx;    // Transmit power (0.0 to 1.0)
    double Prx;    // Receive power (0.0 to 1.0)
    double Pint;   // Interference probability (0.0 to 1.0)
} Config;

Config parse_config(const char* filename);
Config parse_config(const std::string& filename);
Config parse_config_modern(const std::string& filename);
void free_config(Config* config);

// Load configuration from file and populate global simulation context
void load_config();

// Save simulation results to file
// filename: output filename
// collisions: array of collision counts
// rates: array of rates (timings in ms)
// len_rates: length of rates and collisions arrays
// iterations: number of iterations
// error: positional error
// loss: loss probability
// speed: agent speed
// prob: system type ('A'=No loss, 'E'=Wi-Fi, 'C'=ADS-B)
void save_results(const char* filename, double collisions[], const double rates[], 
                  uint32_t len_rates, uint32_t iterations, double error, double loss, 
                  double speed, char prob);
void save_results(const std::string& filename, double collisions[], const double rates[], 
                  uint32_t len_rates, uint32_t iterations, double error, double loss, 
                  double speed, char prob);
#endif
