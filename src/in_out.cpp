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

SimConfig parseInputParameters(const std::vector<std::string>& params){
    SimConfig sc;
    for(auto p:params){
        std::cout<<p<<std::endl;
    }
    return sc;
}

SimConfig readConfigFromFile(const std::string& filename){
    SimConfig sc;
    std::string tmp;
    std::ifstream file(filename);
    //This is really ugly but I don't know how to do it better.
    //For now: good enough!
    while(std::getline(file, tmp)){
        if(tmp.find("//")!=std::string::npos){
            continue;
        }
        std::stringstream line2stream(tmp);
        std::string head;
        line2stream >> head;
        
        if(head == "num_drones"){
            line2stream >> sc.num_drones;
            
        }
        else if(head == "drone_size"){
            line2stream >> sc.drone_size;
            
        }
        else if(head == "drone_speed"){
            line2stream >> sc.drone_speed;
            
        }
        else if(head == "width"){
            line2stream >> sc.width;
            
        }
        else if(head == "height"){
            line2stream >> sc.height;
            
        }
        else if(head == "start_rate"){
            line2stream >> sc.start_rate;
            
        }
        else if(head == "stop_rate"){
            line2stream >> sc.stop_rate;
            
        }
        else if(head == "step_rate"){
            line2stream >> sc.step_rate;
            
        }
        else if(head == "pos_error_dev"){
            line2stream >> sc.pos_error_dev;
            
        }
        else if(head == "height"){
            line2stream >> sc.height;
            
        }
        else if(head == "path_loss_model"){
            line2stream >> sc.path_loss_model;
            
        }
        else if(head == "dt"){
            line2stream >> sc.dt;
            
        }
        else if(head == "iterations"){
            line2stream >> sc.iterations;
            
        }        

    }
    
    return sc;
}