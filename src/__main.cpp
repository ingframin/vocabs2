#include <vector>
#include <iostream>
#include "vec2.h"
#include "drone.h"
#include "comms.h"
#include "in_out.h"


int main(int argc, char *argv[])
{
    //collect input parameters
    std::vector<std::string> params(argv,argv+argc);
    parseInputParameters(params);
    auto config = readConfigFromFile("config.txt");
    
    return 0;
}
