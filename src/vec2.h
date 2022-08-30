#ifndef VEC2_H
#define VEC2_H
//round floats and doubles to int
#define DROUND(x) (int)(x + 0.5)
#include<stdint.h>
typedef struct
{
  double x;
  double y;

} vec2;

typedef struct mat2
{
  double m00;
  double m01;
  double m10;
  double m11;
} mat2x2;


//module
double v2_mod(vec2 v);

//rotate
vec2 v2_rotate(vec2, double angle);

//rotate sign * PI/2
vec2 v2_rotateHalfPI(vec2, int sign);

//rotate PI
vec2 v2_rotatePI(vec2);

//normalize
vec2 v2_norm(vec2);

//Add and subtract
vec2 v2_add(vec2, vec2);
vec2 v2_sub(vec2, vec2);

//dot product
double v2_dot(vec2, vec2);

//add constant to both elements
vec2 v2_addK(vec2, double k);

//multiply both elements by k
vec2 v2_prodK(vec2, double k);

//distance between 2 vectors
double v2_distance(vec2, vec2);

//linear interpolation between 2 points
vec2* v2_lerp(vec2 p1, vec2 p2, uint32_t steps);
#endif
