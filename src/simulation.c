#include "simulation.h"
#include "drone.h"
#include "file_system.h"
#include "string.h"

void parse_config(Text config){
    
    for(int i = 0; i < content.size; i++){

    }

}

Configuration* Conf_newConfiguration(const char* configfile){
    Text content = read_text_file(configfile);
    
}


void Conf_freeConfiguration(Configuration* conf);
void Sim_run(const Configuration* conf);