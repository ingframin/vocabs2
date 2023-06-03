
/* 
Vocabs2 - velocity obstacle for drones simulator
Copyright (C) 2023  Franco Minucci

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef VEC2_H
#define VEC2_H
#include<stdint.h>
#include <stdlib.h>

typedef struct
{
  double x;
  double y;

} vec2;


//module
double v2_mod(vec2 v);

//rotate
vec2 v2_rotate(vec2 v, double angle);

//rotate sign * PI/2
vec2 v2_rotateHalfPI(vec2 v);

//rotate PI
vec2 v2_rotatePI(vec2 v);

//normalize
vec2 v2_normalize(vec2 v);

//Add and subtract
vec2 v2_add(vec2 v1, vec2 v2);
vec2 v2_diff(vec2 v1, vec2 v2);

//dot product
double v2_dot(vec2 v1, vec2 v2);

//add constant to both elements
vec2 v2_addK(vec2 v, double k);

//multiply both elements by k
vec2 v2_scale(vec2 v, double k);

//distance between 2 vectors
double v2_distance(vec2 v1, vec2 v2);

//linear interpolation between 2 points
vec2 v2_lerp(vec2 p1, vec2 p2, double t);

//Quadratic spline
vec2 v2_qspline(vec2 p1, vec2 p2,vec2 p3, double t);

//Cubic spline
vec2 v2_cspline(vec2 p1, vec2 p2, vec2 p3, vec2 p4, double t);

//N-points interpolation
//!!!! Modifies the array vs !!!!
vec2 v2_interpolate(vec2 vs[], size_t vs_len, double t);

//Angle between 2 vectors
double v2_angle_between(vec2 v1, vec2 v2);
#endif
