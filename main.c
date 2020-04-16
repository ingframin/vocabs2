#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include "vec2.h"
#include "drone.h"
Drone d;

int main(int argc, char* argv[]){

  d = DR_newDrone(10.0,10.0,5.0,0.0,2);
  vec2 p1 = {20.0,30.0};
  vec2 p2 = {50.0,20.0};
  vec2 p3 = {20.0,50.0};
  DR_push_waypoint(&d, p3);
  DR_push_waypoint(&d, p2);
  DR_push_waypoint(&d, p1);

  for(int i=0;i<100;i++){
    DR_move(&d,i/100.0);
    printf("%.4f\n",v2_distance(&d.position,&d.waypoints[d.curr_wp]));
    printf("wp: %.2f;%.2f\n",d.waypoints[d.curr_wp].x,d.waypoints[d.curr_wp].y);
    
  }
  double a1 = 3.2;
  double a2 = 3.6;
  printf("%d,%d\n",DROUND(a1),DROUND(a2));
  return 0;

}
