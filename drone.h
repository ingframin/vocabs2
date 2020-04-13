#ifndef DRONE_H
#define DRONE_H
#include "vec2.h"
#include<stdint.h>
#include<stdbool.h>

typedef struct{
  double alpha;
  double beta;
  double gamma;
}barycoords;

typedef struct
{
  //radius
	double radius;
	//center
	vec2 position;
	//tangent points
	vec2 T1;
	vec2 T2;
} Obstacle;

typedef struct {
  uint32_t id;
  vec2 position;
  vec2 speed;
  vec2* waypoints;
  double size;  
} Drone;

void DR_move(Drone* d, double dt);
void DR_goto(Drone* d, vec2 waypoint);
bool DR_collision(Drone* d1, Drone* d2);
void DR_avoid(Drone* d, Drone* d2);
void DR_push_waypoint(Drone* d, vec2 wp);
void DR_pop_waypoint(Drone* d, vec2 wp);

#endif
