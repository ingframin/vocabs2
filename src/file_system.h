
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
#include <stdio.h>
#define FILE_SIZE(fp, size)  fseek(fp,0L,SEEK_END);\
    size = ftell(fp);\
    fseek(fp,0L,SEEK_SET);
#include <stdint.h>
#ifdef __unix
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),(mode)))==NULL
#define strcat_s(dest,size,source) strcat((dest),(source))
#endif

typedef struct content{
    char* text;
    uint32_t size;
}Text;

//Allocates an array on the heap containing the text file content
//!!Remember to free the returned array when done!!!
Text read_text_file(const char* filename);
void write_text_file(const char* filename, const Text* text);

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
} Config;

Config parse_config(const char* filename);
void free_config(Config* config);

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
#endif
