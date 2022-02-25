#include "in_out.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdint>
#include <iomanip>

void saveResults(const std::string& filename, char prob, const std::vector<double>& rates, const std::vector<long>& collisions, double error, double speed, double iterations, double loss){
    std::ofstream results;
    results.open(filename, std::ios_base::app);
    std::stringstream formatted_str; 
    formatted_str << "Error: "<< std::fixed << std::setprecision(3) << error<<"\n";
    formatted_str << "Loss: "<< std::fixed << std::setprecision(3) << loss<<"\n";
    formatted_str << "Speed: "<< std::fixed << std::setprecision(3) << speed<<"\n";
    results<<formatted_str.str();
    
    switch (prob)
    {
    case 'E':
        results<<"Wi-Fi beacons\n";
        break;
    case 'C':
        results<<"ADS-B\n";
        break;
    default:
        results<<"No loss\n";
    }
    std::stringstream formatted_res; 
    for (uint32_t k = 0; k <rates.size(); k++)
    {
        formatted_res<<std::fixed << std::setprecision(3) << rates[k]<<"\t"<<std::fixed << std::setprecision(10)<<collisions[k] / iterations<<"\n";
        
    }
    results<<formatted_res.str()<<std::endl;
    
    results.close();
}