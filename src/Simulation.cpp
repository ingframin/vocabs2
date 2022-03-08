#include "Simulation.h"
#include "vec2.h"
#include "free_space_path_loss.h"
#include <algorithm>
#include <random>

Simulation::Simulation(const SimConfig& sc){
    
    for(uint64_t h = sc.drone_size; h<sc.height/sc.drone_size; h+=2*sc.drone_size){
        for(uint64_t w = sc.drone_size; w<sc.width/sc.drone_size; w+=2*sc.drone_size){
            vec2 p {h, w};
            starting_positions.push_back(p);
        }
    }

    init();

    
}

void Simulation::init(){
    std::random_device rd;
    std::mt19937 g(rd());
 
    std::shuffle(starting_positions.begin(), starting_positions.end(), g);
    auto it = starting_positions.begin();
    
    //!!Assumption is num_drones < num starting positions

    for(uint64_t i = 0; i<sc.num_drones; i++){
        //Need to compute spacial distribution
        vec2 ptmp = *it;
        if(it != starting_positions.end()){
            it++;
        }
        else{
            it = starting_positions.begin();
        }
        
        Drone dtmp(ptmp.x,ptmp.y,sc.drone_speed,0,sc.drone_size);
        state_A.push_back(dtmp);
        state_B.push_back(dtmp);
    }
    //!!Some realistic but arbitrary values to start from
    //TODO: Setup proper path loss model in configuration
    plmodel = FreeSpacePL(2.4e9,20e6,15,10);
}