#ifndef RAND_NUMBERS_H
#define RAND_NUMBERS_H
#include <random>

struct RandomNum{
    
    RandomNum(double scale){
        this->scale = scale;
    }
    
    double getDouble(){
        return scale*rd()/rd.max();
    }
    
     
private:
    double scale;
    std::random_device rd;

};

#endif