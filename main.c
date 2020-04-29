#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<SDL2/SDL.h>


#include "vec2.h"
#include "drone.h"
#include "video.h"

Drone* d1;
Drone* d2;
time_t t;

int main(int argc, char* argv[]){
  t = time(NULL);
  srand(t);
  Display* disp = initVideo(800,800);
  d1 = DR_newDrone(0.0,0.0,20.0,0.0,20);
  d2 = DR_newDrone(800.0,0.0,20.0,0.0,20);
  
  vec2 p1 = {400.0,400.0};
  vec2 p2 = {800.0,800.0};
  vec2 p3 = {000.0,800.0};
  DR_push_waypoint(d1, p1);
  DR_push_waypoint(d2, p1);
  DR_push_waypoint(d1, p2);
  DR_push_waypoint(d2, p3);

  bool running = true;
  
  SDL_Event evt;
  while(running){
    SDL_PumpEvents();
    while(SDL_PollEvent(&evt)){
      if(evt.type==SDL_QUIT){
        running = false;
      }
    }
    DR_move(d1,1e-1);
    DR_move(d2,1e-1);
    clear(disp);
    drawDrone(disp,d1);
    drawDrone(disp,d2);
    render(disp);
    
  }
  
  quitVideo(disp);
  return 0;

}
