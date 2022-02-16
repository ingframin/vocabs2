#ifndef OBSTACLES_H
#define OBSTACLES_H
#include "vec2.h"
//Struct to keep barycentric coordinates
struct Barycoords
{
  double alpha;
  double beta;
  double gamma;
};

struct Obstacle
{
  //radius
  double radius;
  //center
  vec2 position;
  //tangent points
  vec2 T1;
  vec2 T2;
};

Obstacle compute_obstacle(double d1_size, double d2_size, vec2 d1_position, vec2 d2_position);
Barycoords barycentric(vec2 A, vec2 B, vec2 C, vec2 P);
#endif