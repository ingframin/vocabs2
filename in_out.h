#ifndef IN_OUT_H
#define IN_OUT_H
#include <vector>
#include <string>

void saveResults(const std::string& filename, char prob,std::vector<double>& rates, std::vector<long>& collisions, double error, double speed,double iterations, double loss);
#endif