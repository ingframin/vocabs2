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

#ifndef MATH2D_H
#define MATH2D_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// 2D Vector type
typedef struct
{
  double x;
  double y;
} vec2;

// Barycentric coordinates type
typedef struct
{
  double alpha;
  double beta;
  double gamma;
} barycoords;

// Obstacle type (for velocity obstacle calculations)
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

// ======================
// Basic Vector Operations
// ======================

// Vector magnitude (Euclidean norm)
double v2_mod(vec2 v);

// Vector normalization
vec2 v2_normalize(vec2 v);

// Vector addition and subtraction
vec2 v2_add(vec2 v1, vec2 v2);
vec2 v2_diff(vec2 v1, vec2 v2);

// Vector scaling
vec2 v2_scale(vec2 v, double k);

// Add constant to both vector components
vec2 v2_addK(vec2 v, double k);

// Vector dot product
double v2_dot(vec2 v1, vec2 v2);

// ======================
// Geometric Transformations
// ======================

// Vector rotation by angle (radians)
vec2 v2_rotate(vec2 v, double angle);

// Specialized rotations
vec2 v2_rotateLeftHalfPI(vec2 v);
vec2 v2_rotateRightHalfPI(vec2 v);
vec2 v2_reverse(vec2 v);

// ======================
// Distance and Angle Calculations
// ======================

// Distance between two points
double v2_distance(vec2 v1, vec2 v2);

// Angle between two vectors (radians)
double v2_angle_between(vec2 v1, vec2 v2);

// ======================
// Interpolation
// ======================

// Linear interpolation between two points
vec2 v2_lerp(vec2 p1, vec2 p2, double t);

// Quadratic spline interpolation
vec2 v2_qspline(vec2 p1, vec2 p2, vec2 p3, double t);

// Cubic spline interpolation
vec2 v2_cspline(vec2 p1, vec2 p2, vec2 p3, vec2 p4, double t);

// N-points interpolation using de Casteljau's algorithm
// Returns a newly allocated array of points (length vs_len), caller must free
vec2* v2_interpolate(const vec2 vs[], size_t vs_len, double t);

// Free the array returned by v2_interpolate
void v2_free_interpolated(vec2* points);

// ======================
// Barycentric Coordinates
// ======================

// Compute barycentric coordinates of point P with respect to triangle ABC
barycoords v2_barycentric(vec2 A, vec2 B, vec2 C, vec2 P);

// ======================
// Obstacle Calculations (Velocity Obstacle Method)
// ======================

// Compute velocity obstacle between two agents
// pos1, pos2: positions of the two agents
// size1, size2: radii of the two agents
Obstacle compute_obstacle(vec2 pos1, vec2 pos2, double size1, double size2);

// ======================
// Utility Functions
// ======================

// Check if a vector is approximately zero
bool v2_is_zero(vec2 v, double epsilon);

#endif