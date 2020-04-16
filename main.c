#include<stdio.h>
#include<stdlib.h>

#include<SDL2/SDL.h>


#include "vec2.h"
#include "drone.h"
#include "video.h"

Drone* d;


int main(int argc, char* argv[]){
  Display* disp = initVideo(800,800);
  d = DR_newDrone(100.0,100.0,20.0,0.0,20);
  
  vec2 p1 = {80.0,600.0};
  vec2 p2 = {500.0,500.0};
  vec2 p3 = {200.0,500.0};
  vec2 p4 = {600.0,500.0};
  vec2 p5 = {200.0,600.0};
  vec2 p6 = {300.0,500.0};
  DR_push_waypoint(d, p6);
  DR_push_waypoint(d, p5);
  DR_push_waypoint(d, p4);
  DR_push_waypoint(d, p3);
  DR_push_waypoint(d, p2);
  DR_push_waypoint(d, p1);
  bool running = true;
  
  SDL_Event evt;
  while(running){
    SDL_PumpEvents();
    while(SDL_PollEvent(&evt)){
      if(evt.type==SDL_QUIT){
        running = false;
      }
    }
    DR_move(d,1e-1);
    clear(disp);
    drawDrone(disp,d);
    render(disp);
    
  }
  
  quitVideo(disp);
  return 0;

}
