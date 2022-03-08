#ifndef SIMULATION_H
#define SIMULATION_H
#include <vector>
#include <string>
#include "drone.h"
#include "in_out.h"
#include "pathloss.h"
//!! SimConfig should be part of Simulation.h (or should it?)
class Simulation{
public:
    Simulation(const SimConfig& sc);
    void init();
    void run();
    void save();
    void resume(const std::string& path);

private:
    std::vector<Drone> state_A;
    std::vector<Drone> state_B;
    std::vector<vec2> starting_positions;
    PathLoss plmodel;
    uint64_t timer=0;
    double time_scale = 1E-3; //milliseconds
    uint64_t iterations = 1000000;

};
#endif