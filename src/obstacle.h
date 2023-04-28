
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

#ifndef OBSTACLE_H
#define OBSTACLE_H
#include "vec2.h"
//Struct to keep v2_barycentric coordinates
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
barycoords v2_barycentric(vec2 A, vec2 B, vec2 C, vec2 P);
#endif