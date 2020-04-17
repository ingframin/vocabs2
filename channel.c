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

Message CH_receive(Channel* c, double Ploss){
    double n = (1.0*rand()) / RAND_MAX;
    if(n < Ploss){
        return c->messages[c->count];
    }

}