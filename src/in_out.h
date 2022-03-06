#ifndef IN_OUT_H
#define IN_OUT_H
#include <vector>
#include <string>

struct SimConfig{
    uint64_t num_drones = 2;
    double drone_size = 1.0;
    double drone_speed = 20.0;
    uint64_t width = 1000;
    uint64_t height = 1000;
    double start_rate = 1E-3;
    double stop_rate = 20.0;
    double step_rate = 0.25;
    double pos_error_dev = 0;
    std::string path_loss_model = "No Loss";
    double dt = 1E-3;
    uint32_t iterations = 20000;
};

SimConfig readConfigFromFile(const std::string& filename);
void saveResults(const std::string& filename, char prob,const std::vector<double>& rates, const std::vector<long>& collisions, double error, double speed,double iterations, double loss);
SimConfig parseInputParameters(const std::vector<std::string>& params);
#endif