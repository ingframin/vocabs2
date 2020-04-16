#ifndef CHANNEL_H
#define CHANNEL_H
#include<stdint.h>
#include<stdbool.h>
#include "drone.h"
#include "vec2.h"
/*Broadcasts are handled with publish subscribe*/
typedef struct{
    uint32_t id;
    vec2 pos;
    vec2 speed;
}Message;


//How to manage this in such a way that it can be parallelized?

#endif