#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <SDL.h>

#include "vec2.h"
#include "drone.h"
#include "video.h"

Drone *d1;
Drone *d2;
time_t t;
double dt = 0.01;

int main(int argc, char *argv[])
{
  t = time(NULL);
  srand(t);
  Display *disp = initVideo(800, 800);
  d1 = DR_newDrone(0.0, 0.0, 20.0, 0.0, 20);
  d2 = DR_newDrone(800.0, 0.0, 20.0, 0.0, 20);

  vec2 p1 = {400.0, 400.0};
  vec2 p2 = {800.0, 800.0};
  vec2 p3 = {000.0, 800.0};
  DR_push_waypoint(d1, p1);
  DR_push_waypoint(d2, p1);
  DR_push_waypoint(d1, p2);
  DR_push_waypoint(d2, p3);

  bool running = true;

  SDL_Event evt;
  while (running)
  {
    SDL_PumpEvents();
    while (SDL_PollEvent(&evt))
    {
      if (evt.type == SDL_QUIT)
      {
        running = false;
      }
    }
    DR_avoid(d2, d1);
    DR_avoid(d1, d2);
    DR_move(d1, dt);
    DR_move(d2, dt);
    if (d1->waypoints[d1->curr_wp].x == 0 && d1->waypoints[d1->curr_wp].y == 0)
    {
      running = false;
    }
    if (d2->waypoints[d2->curr_wp].x == 0 && d2->waypoints[d2->curr_wp].y == 0)
    {
      running = false;
    }
    clear(disp);
    drawDrone(disp, d1);
    drawDrone(disp, d2);
    render(disp);
  }

  quitVideo(disp);
  return 0;
}
