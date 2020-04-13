#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include "vec2.h"
#include "drone.h"
Drone d;

int main(int argc, char* argv[]){

  d.position.x = 100;
  d.position.y = 100;
  d.speed.x = 10;
  d.speed.y = 0;
  vec2 waypoint;
  waypoint.x = 500;
  waypoint.y = -200;
  printf("%.6f;%.6f\n",d.speed.x,d.speed.y);
  DR_goto(&d,waypoint);
  printf("%.6f;%.6f\n",d.speed.x,d.speed.y);
  return 0;

}
