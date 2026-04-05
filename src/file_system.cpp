
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

#include "file_system.h"
#include <cstdlib>
#include <cstring>
#include <ctype.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>

#define FILE_SIZE(fp, size)  fseek(fp,0L,SEEK_END);\
    size = ftell(fp);\
    fseek(fp,0L,SEEK_SET);

// Global simulation context (defined here, declared in simulation_context.h)
SimulationContext sim_context = {
    .iterations = 0,
    .dt = 0.0,
    .error = 0.0,
    .rates = NULL,
    .num_threads = 0,
    .speed = 0.0,
    .si = 0,
    .rate = 1,
    .prob = 'A',
    .sys = NO_LOSS,
    .l = 0.0,
    .len_rates = 0
};

Text read_text_file(const char* filename){
    return read_text_file_legacy(std::string(filename));
}

Text read_text_file_legacy(const std::string& filename){
    //Open file and create a buffer of the corresponding size
    FILE* fp;
    fopen_s(&fp, filename.c_str(), "r");
    int size;
    FILE_SIZE(fp,size);
    std::cout << size << "\n";
    //size+1 to be able to add a '\0' at the end
    Text cnt;
    cnt.size = size;
    cnt.text = (char*)calloc(sizeof(char)*(size+1),size+1);
    char buffer[1024];
    //Read content
    while(fgets(buffer, 1024, fp)!=NULL){
        strcat_s(cnt.text,size,buffer);
    };
    
    cnt.text[size] = '\0';
    fclose(fp);

    return cnt;
}

void write_text_file(const char* filename, const Text* text) {
    write_text_file_legacy(std::string(filename), text);
}

void write_text_file_legacy(const std::string& filename, const Text* text) {
    FILE* fp; 
    fopen_s(&fp, filename.c_str(), "w");
    if (fp == NULL) {
        std::cerr << "Error: could not open file " << filename << " for writing\n";
        exit(1);
    }

    size_t len = text->size;
    size_t bytes_written = fwrite(text->text, sizeof(char), len, fp);
    if (bytes_written != len) {
        std::cerr << "Error: could not write entire file " << filename << "\n";
        exit(1);
    }

    fclose(fp);
}

// Modern C++ version using STL
std::string read_text_file_modern(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: could not open file " << filename << " for reading\n";
        return "";
    }

    // Read entire file into string
    std::stringstream buffer;
    buffer << file.rdbuf();
    
    file.close();
    return buffer.str();
}

// Modern C++ version for writing text files
void write_text_file_modern(const std::string& filename, const std::string& content) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: could not open file " << filename << " for writing\n";
        return;
    }

    file << content;
    
    if (!file.good()) {
        std::cerr << "Error: could not write entire file " << filename << "\n";
    }

    file.close();
}

// Trim whitespace from the start and end of a string
static void trim_whitespace(char* str) {
    if (str == NULL || *str == '\0') {
        return;
    }

    // Trim leading whitespace
    char* start = str;
    while (isspace((unsigned char)*start)) {
        start++;
    }

    // Trim trailing whitespace
    char* end = str + strlen(str) - 1;
    while (end > start && isspace((unsigned char)*end)) {
        end--;
    }
    *(end + 1) = '\0';

    // Move the trimmed string to the start of the buffer
    memmove(str, start, end - start + 2);
}

// Parse a line in the format "key = value"
static int parse_key_value(const char* line, char* key, char* value) {
    const char* equals = strchr(line, '=');
    if (equals == NULL) {
        return 0; // No '=' found
    }

    strncpy(key, line, static_cast<size_t>(equals - line));
    key[equals - line] = '\0';
    trim_whitespace(key);

    strncpy(value, equals + 1, 255);
    value[255] = '\0';
    trim_whitespace(value);

    return 1; // Success
}

// Parse a double array from a comma-separated string
static double* parse_double_array(const char* str, int* count) {
    int capacity = 10;
    double* array = (double*)malloc(capacity * sizeof(double));
    if (array == NULL) {
        return NULL;
    }

    char* token = strtok((char*)str, ",");
    int i = 0;
    while (token != NULL) {
        if (i >= capacity) {
            capacity *= 2;
            double* new_array = (double*)realloc(array, capacity * sizeof(double));
            if (new_array == NULL) {
                free(array);
                return NULL;
            }
            array = new_array;
        }

        array[i++] = atof(token);
        token = strtok(NULL, ",");
    }

    *count = i;
    return array;
}

Config parse_config(const char* filename) {
    return parse_config(std::string(filename));
}

Config parse_config(const std::string& filename) {
    Config config = {0};

    FILE* fp;
    fopen_s(&fp, filename.c_str(), "r");
    if (fp == NULL) {
        std::cerr << "Error: could not open config file " << filename << "\n";
        exit(1);
    }

    char line[256];
    char current_section[64] = "";
    char key[64];
    char value[256];

    while (fgets(line, sizeof(line), fp) != NULL) {
        trim_whitespace(line);

        // Skip empty lines and comments
        if (line[0] == '\0' || line[0] == ';' || line[0] == '#') {
            continue;
        }

        // Check for section headers
        if (line[0] == '[') {
            char* end = strchr(line, ']');
            if (end != NULL) {
                strncpy(current_section, line + 1, end - line - 1);
                current_section[end - line - 1] = '\0';
                trim_whitespace(current_section);
            }
            continue;
        }

        // Parse key-value pairs
        if (parse_key_value(line, key, value)) {
            if (strcmp(current_section, "Simulation") == 0) {
                if (strcmp(key, "iterations") == 0) {
                    config.iterations = atoi(value);
                } else if (strcmp(key, "dt") == 0) {
                    config.dt = atof(value);
                }
            } else if (strcmp(current_section, "Error") == 0) {
                if (strcmp(key, "error") == 0) {
                    config.error = atof(value);
                }
            } else if (strcmp(current_section, "Rates") == 0) {
                if (strcmp(key, "num_rates") == 0) {
                    config.num_rates = atoi(value);
                } else if (strcmp(key, "rates") == 0) {
                    config.rates = parse_double_array(value, (int*)&config.num_rates);
                }
            } else if (strcmp(current_section, "Threads") == 0) {
                if (strcmp(key, "num_threads") == 0) {
                    config.num_threads = atoi(value);
                }
            } else if (strcmp(current_section, "Speed") == 0) {
                if (strcmp(key, "speed") == 0) {
                    config.speed = atof(value);
                }
            } else if (strcmp(current_section, "Drones") == 0) {
                if (strcmp(key, "num_drones") == 0) {
                    config.num_drones = atoi(value);
                }
            } else if (strcmp(current_section, "System") == 0) {
                if (strcmp(key, "prob") == 0) {
                    config.prob = value[0];
                } else if (strcmp(key, "loss") == 0) {
                    config.loss = atof(value);
                }
            } else if (strcmp(current_section, "Communication") == 0) {
                if (strcmp(key, "Ptx") == 0) {
                    config.Ptx = atof(value);
                } else if (strcmp(key, "Prx") == 0) {
                    config.Prx = atof(value);
                } else if (strcmp(key, "Pint") == 0) {
                    config.Pint = atof(value);
                }
            }
        }
    }

    fclose(fp);
    return config;
}

// Modern C++ version of config parsing using STL
Config parse_config_modern(const std::string& filename) {
    Config config = {0};

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: could not open config file " << filename << "\n";
        exit(1);
    }

    std::string current_section = "";
    std::string line;
    
    while (std::getline(file, line)) {
        // Trim whitespace
        size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) continue; // Empty line
        size_t end = line.find_last_not_of(" \t");
        line = line.substr(start, end - start + 1);

        // Skip empty lines and comments
        if (line.empty() || line[0] == ';' || line[0] == '#') {
            continue;
        }

        // Check for section headers
        if (line[0] == '[' && line.back() == ']') {
            current_section = line.substr(1, line.length() - 2);
            continue;
        }

        // Parse key-value pairs
        size_t equals_pos = line.find('=');
        if (equals_pos != std::string::npos) {
            std::string key = line.substr(0, equals_pos);
            std::string value = line.substr(equals_pos + 1);
            
            // Trim whitespace from key and value
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);

            if (current_section == "Simulation") {
                if (key == "iterations") {
                    config.iterations = std::stoi(value);
                } else if (key == "dt") {
                    config.dt = std::stod(value);
                }
            } else if (current_section == "Error") {
                if (key == "error") {
                    config.error = std::stod(value);
                }
            } else if (current_section == "Rates") {
                if (key == "num_rates") {
                    config.num_rates = std::stoi(value);
                } else if (key == "rates") {
                    // Parse comma-separated rates
                    std::stringstream ss(value);
                    std::string token;
                    std::vector<double> rates;
                    while (std::getline(ss, token, ',')) {
                        rates.push_back(std::stod(token));
                    }
                    if (!rates.empty()) {
                        config.rates = (double*)malloc(rates.size() * sizeof(double));
                        for (size_t i = 0; i < rates.size(); i++) {
                            config.rates[i] = rates[i];
                        }
                        config.num_rates = rates.size();
                    }
                }
            } else if (current_section == "Threads") {
                if (key == "num_threads") {
                    config.num_threads = std::stoi(value);
                }
            } else if (current_section == "Speed") {
                if (key == "speed") {
                    config.speed = std::stod(value);
                }
            } else if (current_section == "Drones") {
                if (key == "num_drones") {
                    config.num_drones = std::stoi(value);
                }
            } else if (current_section == "System") {
                if (key == "prob") {
                    config.prob = value.empty() ? 'A' : value[0];
                } else if (key == "loss") {
                    config.loss = std::stod(value);
                }
            } else if (current_section == "Communication") {
                if (key == "Ptx") {
                    config.Ptx = std::stod(value);
                } else if (key == "Prx") {
                    config.Prx = std::stod(value);
                } else if (key == "Pint") {
                    config.Pint = std::stod(value);
                }
            }
        }
    }

    file.close();
    return config;
}

void free_config(Config* config) {
    if (config->rates != NULL) {
        free(config->rates);
        config->rates = NULL;
    }
}

void save_results(const char* filename, double collisions[], const double rates[], 
                  uint32_t len_rates, uint32_t iterations, double error, double loss, 
                  double speed, char prob) {
    save_results(std::string(filename), collisions, rates, len_rates, iterations, error, loss, speed, prob);
}

// Modern C++ version using STL
void save_results(const std::string& filename, double collisions[], const double rates[], 
                  uint32_t len_rates, uint32_t iterations, double error, double loss, 
                  double speed, char prob) {
    std::ofstream results(filename, std::ios::app);
    if (!results.is_open()) {
        std::cerr << "Error: could not open file " << filename << " for writing\n";
        return;
    }

    results << "Error: " << std::fixed << std::setprecision(3) << error << "\n";
    results << "Loss: " << std::fixed << std::setprecision(3) << loss << "\n";
    results << "Speed: " << std::fixed << std::setprecision(3) << speed << "\n";
    
    switch (prob)
    {
    case 'E':
        results << "Wi-Fi beacons\n";
        break;
    case 'C':
        results << "ADS-B\n";
        break;
    default:
        results << "No loss\n";
    }

    for (uint32_t k = 0; k < len_rates; k++)
    {
        std::cout << std::fixed << std::setprecision(3) << (1000.0/rates[k]) 
                 << "\t" << std::fixed << std::setprecision(6) << (collisions[k] / iterations) << "\n";
        results << std::fixed << std::setprecision(3) << (1000.0/rates[k]) 
               << "\t" << std::fixed << std::setprecision(10) << (collisions[k] / iterations) << "\n";
    }
    results.close();
}

void load_config() {
    Config config = parse_config("config.ini");
    
    sim_context.iterations = config.iterations;
    sim_context.dt = config.dt;
    sim_context.error = config.error;
    
    // Copy the rates array - we need to manage our own copy
    if (config.rates != NULL && config.num_rates > 0) {
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
    // Set communication parameters
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
    
    // Free the config's rates array since we've made our own copy
    free_config(&config);
}