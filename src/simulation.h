#ifndef SIMULATION_H
#define SIMULATION_H
#include "drone.h"
#include "vec2.h"

typedef struct config{
    Drone* drones;
    uint64_t num_drones;
    vec2* points;
    uint64_t num_points;

} Configuration;

Conf_pushDrone(double x, double y, double vx, double vy, double size);
Conf_pushPoints(double x, double y, double vx, double vy, double size);
#endif
