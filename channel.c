#include "channel.h"
#include <math.h>
#include <stdlib.h>

Channel* CH_newChannel(unsigned int length){
    Channel* c = malloc(sizeof(Channel));
    c->len = length;
    c->count = 0;
    c->messages = malloc(length*sizeof(Message));
    return c;
}

void CH_freeChannel(Channel* c){
    free(c->messages);
    free(c);
}
void CH_broadcast(Channel* c, Message m){
    if(c->count+1 < c->len){
        c->messages[c->count] = m;
        c->count += 1;
    }
}

Message CH_receive(Channel* c, double Ploss, unsigned int index){
    double n = (1.0*rand()) / RAND_MAX;
    if(n < Ploss && index < c->count){
        return c->messages[index];
    }
    Message m;
    m.size = -1;//Negative size for a bad message
    return m;
}