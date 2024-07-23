#include "simulation.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <omp.h>
#include <time.h>
#include "vec2.h"
#include "drone.h"
#include "obstacle.h"
#include "flightplan.h"
#include "comms.h"
#include "text.h"


Configuration* create_configuration_from_file(const char* file_path) {
    Text content = read_text_file(file_path);
    Configuration* configuration = create_configuration(content);
    free_text(content);
    return configuration;
}

Configuration* create_configuration(Text content) {
    char* drones_start = strstr(content.content, "[drones:");
    char* drones_end = strstr(drones_end, "]");
    char* points_start = strstr(content.content, "[points:");
    char* points_end = strstr(points_end, "]");

    uint64_t num_drones = count_tokens(drones_start, drones_end);
    Drone* drones = create_drones(num_drones, drones_start, drones_end);

    uint64_t num_points = count_tokens(points_start, points_end);
    vec2* points = create_points(num_points, points_start, points_end);

    Configuration* configuration = create_configuration(drones, num_drones, points, num_points);
    free_drones(drones);
    free_points(points);
    return configuration;
}

uint64_t count_tokens(char* start, char* end) {
    uint64_t count = 0;
    char* token = strtok(start, ",");
    while (token != NULL) {
        count++;
        token = strtok(NULL, ",");
    }
    return count;
}

Drone* create_drones(uint64_t num_drones, char* start, char* end) {
    Drone* drones = malloc(num_drones * sizeof(Drone));
    if (!drones) {
        fprintf(stderr, "Failed to allocate memory for drones\n");
        exit(EXIT_FAILURE);
    }

    char* token = strtok(start, "[");
    for (uint64_t i = 0; i < num_drones; i++) {
        drones[i] = create_drone(token, i);
        token = strtok(NULL, ",");
    }

    return drones;
}

Drone* create_drone(char* token, uint64_t id) {
    char* position_start = strstr(token, "(");
    char* position_end = strstr(position_start, ")");
    char* velocity_start = strstr(token, "[");
    char* velocity_end = strstr(velocity_start, "]");
    double size = strtod(strstr(token, "|"), NULL);

    vec2 position = create_vec2(position_start, position_end);
    vec2 velocity = create_vec2(velocity_start, velocity_end);

    Drone* drone = malloc(sizeof(Drone));
    if (!drone) {
        fprintf(stderr, "Failed to allocate memory for drone\n");
        exit(EXIT_FAILURE);
    }

    drone->id = id;
    drone->position = position;
    drone->velocity = velocity;
    drone->size = size;
    return drone;
}

vec2* create_points(uint64_t num_points, char* start, char* end) {
    vec2* points = malloc(num_points * sizeof(vec2));
    if (!points) {
        fprintf(stderr, "Failed to allocate memory for points\n");
        exit(EXIT_FAILURE);
    }

    char* token = strtok(start, "[");
    for (uint64_t i = 0; i < num_points; i++) {
        points[i] = create_vec2(token, NULL);
        token = strtok(NULL, ",");
    }

    return points;
}

vec2* create_vec2(char* token, char* end) {
    double x = strtod(token, NULL);
    double y = strtod(strstr(token, "|"), NULL);

    vec2* vec = malloc(sizeof(vec2));
    if (!vec) {
        fprintf(stderr, "Failed to allocate memory for vec2\n");
        exit(EXIT_FAILURE);
    }

    vec->x = x;
    vec->y = y;

    return vec;
}