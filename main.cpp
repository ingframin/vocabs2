#include "vec2.h"
#include "drone.h"
#include "comms.h"
#include "in_out.h"

#include <iostream>


int main(){
    std::cout<<"Hello World!"<<std::endl;
    vec2 v1 {10.0,20.0};
    vec2 v2 {20.0,10.0};
    auto v3 = v1.sub(v2);
    std::cout <<v3.x<<'\t'<<v3.y<< std::endl;

}