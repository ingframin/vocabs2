#ifndef OBSTACLE_H
#define OBSTACLE_H
//Struct to keep barycentric coordinates
typedef struct
{
  double alpha;
  double beta;
  double gamma;
} barycoords;

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

Obstacle compute_obstacle(vec2 pos1, vec2 pos2, double size1, double size2);
barycoords barycentric(vec2 A, vec2 B, vec2 C, vec2 P);
#endif