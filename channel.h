#ifndef CHANNEL_H
#define CHANNEL_H
#include<stdint.h>
#include<stdbool.h>
#include "drone.h"
#include "vec2.h"
/*Broadcasts are handled with publish subscribe*/
typedef struct{
    uint32_t id;
    uint32_t drone_id;
    vec2 pos;
    vec2 speed;
    double size;
}Message;

typedef struct{
    Message* messages;
    unsigned int len;
    unsigned int count;
}Channel;

Channel* CH_newChannel(unsigned int length);
void CH_freeChannel(Channel* c);
void CH_broadcast(Channel* c,Message m);
Message CH_receive(Channel* c, double Ploss, unsigned int index);

#endif