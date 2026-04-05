
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
    .rates = {},
    .num_threads = 0,
    .speed = 0.0,
    .si = 0,
    .rate = 1,
    .prob = 'A',
    .sys = RFsystem::NO_LOSS,
    .l = 0.0,
    .Ptx = 0.0,
    .Prx = 0.0,
    .Pint = 0.0,
    .filename = ""
};



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

// Trim whitespace from the start and end of a string (C-style)
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

// Trim whitespace from the start and end of a string (C++ style)
static void trim_whitespace(std::string& str) {
    if (str.empty()) {
        return;
    }

    // Trim leading whitespace
    size_t start = str.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) {
        str.clear();
        return;
    }

    // Trim trailing whitespace
    size_t end = str.find_last_not_of(" \t\n\r");
    str = str.substr(start, end - start + 1);
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
    Config config = {
        .iterations = 0,
        .dt = 0.0,
        .error = 0.0,
        .rates = NULL,
        .num_rates = 0,
        .num_threads = 0,
        .speed = 0.0,
        .prob = '\0',
        .loss = 0.0,
        .num_drones = 0,
        .Ptx = 0.0,
        .Prx = 0.0,
        .Pint = 0.0,
        .filename = ""
    };

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: could not open config file " << filename << "\n";
        exit(1);
    }

    std::string line;
    std::string current_section = "";
    std::string key;
    std::string value;

    while (std::getline(file, line)) {
        trim_whitespace(line);

        // Skip empty lines and comments
        if (line.empty() || line[0] == ';' || line[0] == '#') {
            continue;
        }

        // Check for section headers
        if (line[0] == '[' && line.back() == ']') {
            current_section = line.substr(1, line.length() - 2);
            trim_whitespace(current_section);
            continue;
        }

        // Parse key-value pairs
        size_t equals_pos = line.find('=');
        if (equals_pos != std::string::npos) {
            key = line.substr(0, equals_pos);
            value = line.substr(equals_pos + 1);
            trim_whitespace(key);
            trim_whitespace(value);
            if (current_section == "Simulation") {
                if (key == "iterations") {
                    config.iterations = atoi(value.c_str());
                } else if (key == "dt") {
                    config.dt = atof(value.c_str());
                }
            } else if (current_section == "Error") {
                if (key == "error") {
                    config.error = atof(value.c_str());
                }
            } else if (current_section == "Rates") {
                if (key == "num_rates") {
                    config.num_rates = atoi(value.c_str());
                } else if (key == "rates") {
                    config.rates = parse_double_array(value.c_str(), (int*)&config.num_rates);
                }
            } else if (current_section == "Threads") {
                if (key == "num_threads") {
                    config.num_threads = atoi(value.c_str());
                }
            } else if (current_section == "Speed") {
                if (key == "speed") {
                    config.speed = atof(value.c_str());
                }
            } else if (current_section == "Drones") {
                if (key == "num_drones") {
                    config.num_drones = atoi(value.c_str());
                }
            } else if (current_section == "System") {
                if (key == "prob") {
                    config.prob = value.empty() ? '\0' : value[0];
                } else if (key == "loss") {
                    config.loss = atof(value.c_str());
                }
            } else if (current_section == "Communication") {
                if (key == "Ptx") {
                    config.Ptx = atof(value.c_str());
                } else if (key == "Prx") {
                    config.Prx = atof(value.c_str());
                } else if (key == "Pint") {
                    config.Pint = atof(value.c_str());
                }
            } else if (current_section == "Output") {
                if (key == "filename") {
                    // Use string directly
                    config.filename = value;
                }
            }
        }
    }

    file.close();
    return config;
}

// Modern C++ version of config parsing using STL
Config parse_config_modern(const std::string& filename) {
    Config config = {
        .iterations = 0,
        .dt = 0.0,
        .error = 0.0,
        .rates = NULL,
        .num_rates = 0,
        .num_threads = 0,
        .speed = 0.0,
        .prob = '\0',
        .loss = 0.0,
        .num_drones = 0,
        .Ptx = 0.0,
        .Prx = 0.0,
        .Pint = 0.0,
        .filename = ""
    };

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
            } else if (current_section == "Output") {
                if (key == "filename") {
                    // Use string directly
                    config.filename = value;
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
    // filename is now std::string, no need to free
}

void save_results(const char* filename, double collisions[], const std::vector<double>& rates, 
                  uint32_t iterations, double error, double loss, 
                  double speed, char prob) {
    save_results(std::string(filename), collisions, rates, iterations, error, loss, speed, prob);
}

// Modern C++ version using STL
void save_results(const std::string& filename, double collisions[], const std::vector<double>& rates, 
                  uint32_t iterations, double error, double loss, 
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

    for (uint32_t k = 0; k < rates.size(); k++)
    {
        std::cout << std::fixed << std::setprecision(3) << (1000.0/rates[k]) 
                 << "\t" << std::fixed << std::setprecision(6) << (collisions[k] / iterations) << "\n";
        results << std::fixed << std::setprecision(3) << (1000.0/rates[k]) 
               << "\t" << std::fixed << std::setprecision(10) << (collisions[k] / iterations) << "\n";
    }
    results.close();
}

// Add command line argument for output filename
void add_output_filename_argument(int argc, char *argv[], std::string& output_filename) {
    // Default to empty string (will use config file setting)
    output_filename = "";
    
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-o") == 0 || strcmp(argv[i], "--output") == 0) {
            if (i + 1 < argc) {
                output_filename = argv[++i];
            }
        }
    }
}

void load_config() {
    Config config = parse_config("config.ini");
    
    sim_context.iterations = config.iterations;
    sim_context.dt = config.dt;
    sim_context.error = config.error;
    
    // Copy the rates array - we need to manage our own copy
    if (config.rates != NULL && config.num_rates > 0) {
        sim_context.rates.assign(config.rates, config.rates + config.num_rates);
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
        sim_context.sys = RFsystem::WI_FI;
        break;
    case 'C':
        sim_context.sys = RFsystem::ADS_B;
        break;
    default:
        sim_context.sys = RFsystem::NO_LOSS;
    }
    
    // Free the config's rates array since we've made our own copy
    free_config(&config);
}