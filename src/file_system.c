
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

Text read_text_file(const char* filename){
    //Open file and create a buffer of the corresponding size
    FILE* fp;
    fopen_s(&fp,filename,"r");
    int size;
    FILE_SIZE(fp,size);
    printf("%d\n",size);
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
    FILE* fp; 
    fopen_s(&fp,filename, "w");
    if (fp == NULL) {
        fprintf(stderr, "Error: could not open file %s for writing\n", filename);
        exit(1);
    }

    size_t len = text->size;
    size_t bytes_written = fwrite(text->text, sizeof(char), len, fp);
    if (bytes_written != len) {
        fprintf(stderr, "Error: could not write entire file %s\n", filename);
        exit(1);
    }

    fclose(fp);
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
    char* equals = strchr(line, '=');
    if (equals == NULL) {
        return 0; // No '=' found
    }

    strncpy(key, line, equals - line);
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
    Config config = {0};

    FILE* fp;
    fopen_s(&fp, filename, "r");
    if (fp == NULL) {
        fprintf(stderr, "Error: could not open config file %s\n", filename);
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
            } else if (strcmp(current_section, "System") == 0) {
                if (strcmp(key, "prob") == 0) {
                    config.prob = value[0];
                } else if (strcmp(key, "loss") == 0) {
                    config.loss = atof(value);
                }
            }
        }
    }

    fclose(fp);
    return config;
}

void free_config(Config* config) {
    if (config->rates != NULL) {
        free(config->rates);
        config->rates = NULL;
    }
}