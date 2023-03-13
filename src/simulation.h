#ifndef SIMULATION_H
#define SIMULATION_H
#include "drone.h"
#include "vec2.h"

//This structure contains the status of the Simulation
typedef struct config{
    Drone* drones;//This is the base of a list
    uint64_t num_drones;
    vec2* points;
    uint64_t num_points;

} Configuration;

void Conf_pushDrone(double x, double y, double vx, double vy, double size);
void Conf_pushPoint(vec2 point);

void Sim_run(const Configuration* const conf);

#endif
