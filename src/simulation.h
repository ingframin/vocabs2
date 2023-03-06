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

void Conf_pushDrone(double x, double y, double vx, double vy, double size);
void Conf_pushPoint(vec2 point);

void Sim_run(const Configuration* const conf);

#endif
