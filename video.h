#ifndef VIDEO_H
#define VIDEO_H

#include<SDL2/SDL.h>
#include "drone.h"
typedef struct{
    SDL_Window* wnd;
    SDL_Renderer* rnd;    
    int width;
    int height;
}Display;

Display* initVideo(int width,int height);
void quitVideo(Display* d);
void clear(Display* d);
void render(Display* d);
void drawDrone(Display*d, Drone* dr);
#endif