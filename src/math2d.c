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

#include "math2d.h"
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif
#include <stdlib.h>

// ======================
// Basic Vector Operations
// ======================

double v2_mod(vec2 v)
{
  return hypot(v.x, v.y);
}

vec2 v2_normalize(vec2 v)
{
  if(v2_is_zero(v, 1e-12)) {
    // Return zero vector for zero-length input
    return (vec2){0.0, 0.0};
  }
  
  double m = v2_mod(v);
  return (vec2){v.x / m, v.y / m};
}

vec2 v2_add(vec2 v1, vec2 v2)
{
  vec2 res;
  res.x = v1.x + v2.x;
  res.y = v1.y + v2.y;
  return res;
}

vec2 v2_diff(vec2 v1, vec2 v2)
{
  vec2 res;
  res.x = v1.x - v2.x;
  res.y = v1.y - v2.y;
  return res;
}

vec2 v2_scale(vec2 v, double k)
{
  vec2 res;
  res.x = v.x * k;
  res.y = v.y * k;
  return res;
}

vec2 v2_addK(vec2 v, double k)
{
  vec2 res;
  res.x = v.x + k;
  res.y = v.y + k;
  return res;
}

double v2_dot(vec2 v1, vec2 v2)
{
  return v1.x * v2.x + v1.y * v2.y;
}

// ======================
// Geometric Transformations
// ======================

vec2 v2_rotate(vec2 v, double angle)
{
  /*
  * Preliminary checks:
    - if the angle is very small, no need to rotate
    - if the angle is close to PI, simply reverse the vector
    - if the angle is close to pi/2 rotate by pi/2
    - Otherwise, calculate the actual rotation.
    Rotating by small angles close to extremes (pi, pi/2) 
    can cause numerical instability because of floating point errors.

  */
  if(fabs(angle) < 1e-4){
     return v;
  }
  else if(fabs(angle - M_PI) < 1e-4){
    return v2_reverse(v);
  }
  if(fabs(angle - M_PI/2) < 1e-4){
    return v2_rotateLeftHalfPI(v);
  }
  else if(fabs(angle + M_PI/2) < 1e-4){
    return v2_reverse(v2_rotateLeftHalfPI(v));
  }
  
  if(v2_is_zero(v, 1e-12)) {
    // Cannot rotate zero vector
    return v;
  }
  
  double C = cos(angle);
  double S = sin(angle);
  double m = v2_mod(v);
  
  vec2 vn = v2_normalize(v);
  vec2 ret;
  ret.x = m * (vn.x * C - vn.y * S);
  ret.y = m * (vn.x * S + vn.y * C);

  return ret;
}

vec2 v2_rotateLeftHalfPI(vec2 v)
{
  vec2 vr;
  vr.x = -v.y;
  vr.y = v.x;
  return vr;
}

vec2 v2_rotateRightHalfPI(vec2 v)
{
  vec2 vr;
  vr.x = v.y;
  vr.y = -v.x;
  return vr;
}

vec2 v2_reverse(vec2 p){
  vec2 ret;
  ret.x = -p.x;
  ret.y = -p.y;
  return ret;
}

// ======================
// Distance and Angle Calculations
// ======================

double v2_distance(vec2 v1, vec2 v2)
{
  vec2 res = v2_diff(v1, v2);
  return v2_mod(res);
}

double v2_angle_between(vec2 v1, vec2 v2){
  double mod1 = v2_mod(v1);
  double mod2 = v2_mod(v2);
  double mods = mod1 * mod2;
  
  // Handle zero vectors
  if(mods < 1e-12) {
    return 0.0; // Return 0 angle if either vector is zero
  }
  
  double sina = (v1.x*v2.y - v2.x*v1.y)/mods;
  double cosa = (v1.x*v2.x + v1.y*v2.y)/mods;
  
  // Clamp values to avoid domain errors in atan2
  sina = fmin(fmax(sina, -1.0), 1.0);
  cosa = fmin(fmax(cosa, -1.0), 1.0);

  return atan2(sina, cosa);
}

// ======================
// Interpolation
// ======================

vec2 v2_lerp(vec2 p1, vec2 p2, double t){
  vec2 a = v2_scale(p1, 1.0 - t);
  vec2 b = v2_scale(p2, t);
  return v2_add(a, b);
}

vec2 v2_qspline(vec2 p1, vec2 p2, vec2 p3, double t){
    vec2 p0i = v2_lerp(p1, p2, t);
    vec2 p1i = v2_lerp(p2, p3, t);
    return v2_lerp(p0i, p1i, t);
}

vec2 v2_cspline(vec2 p1, vec2 p2, vec2 p3, vec2 p4, double t){
    vec2 p0i = v2_lerp(p1, p2, t);
    vec2 p1i = v2_lerp(p2, p3, t);
    vec2 p2i = v2_lerp(p3, p4, t);
    return v2_qspline(p0i, p1i, p2i, t);
}

vec2* v2_interpolate(const vec2 vs[], size_t vs_len, double t) {
    if (vs_len == 0) return NULL;
    if (!vs) return NULL;  // Add null pointer check
    if (t < 0.0 || t > 1.0) {
        // Clamp t to valid range
        t = fmax(0.0, fmin(t, 1.0));
    }

    // Allocate a new array for the results
    vec2* temp = (vec2*)malloc(vs_len * sizeof(vec2));
    if (!temp) return NULL;

    // Copy input points to temp array
    for (size_t i = 0; i < vs_len; ++i) {
        temp[i] = vs[i];
    }

    // Perform interpolation using de Casteljau's algorithm
    for (size_t level = 1; level < vs_len; ++level) {
        for (size_t i = 0; i < vs_len - level; ++i) {
            temp[i] = v2_lerp(temp[i], temp[i + 1], t);
        }
    }

    return temp;
}

void v2_free_interpolated(vec2* points) {
    if (points) {
        free(points);
    }
}

// ======================
// Barycentric Coordinates
// ======================

barycoords v2_barycentric(vec2 A, vec2 B, vec2 C, vec2 P)
{
	barycoords bc;
	bc.gamma = ((A.y - B.y) * P.x + (B.x - A.x) * P.y + A.x * B.y - B.x * A.y) /
	           ((A.y - B.y) * C.x + (B.x - A.x) * C.y + A.x * B.y - B.x * A.y);
	bc.beta = ((A.y - C.y) * P.x + (C.x - A.x) * P.y + A.x * C.y - C.x * A.y) /
	          ((A.y - C.y) * B.x + (C.x - A.x) * B.y + A.x * C.y - C.x * A.y);
	bc.alpha = 1 - bc.beta - bc.gamma;

	return bc;
}

// ======================
// Obstacle Calculations (Velocity Obstacle Method)
// ======================

Obstacle compute_obstacle(vec2 pos1, vec2 pos2, double size1, double size2)
{
	// Minkowski addition
	double r = size1 + size2;

	//Computing tangent lines to circle passing through the point self.position
	double dx = pos1.x - pos2.x;
	double a = dx * dx - r * r;
	double b = 2 * dx * (pos1.y - pos2.y);
	double c = (pos2.y - pos1.y) * (pos2.y - pos1.y) - r * r;
	double Delta = b * b - 4 * a * c;

	//Angular coefficient
	double m1 = (-b + sqrt(Delta)) / (2 * a);
	double m2 = (-b - sqrt(Delta)) / (2 * a);
	//Intersection with y axis
	double q1 = pos1.y - m1 * pos1.x;
	double q2 = pos1.y - m2 * pos1.x;

	//(xt1,yt1) - first tangent point.
	double a1 = 1 + m1 * m1;
	double b1 = 2 * m1 * q1 - 2 * pos2.x - m1 * 2 * pos2.y;

	double xt1 = (-b1) / (2 * a1);
	double yt1 = m1 * xt1 + q1;

	//(xt2,yt2) - Second tangent point
	double a2 = 1 + m2 * m2;
	double b2 = 2 * m2 * q2 - 2 * pos2.x - m2 * 2 * pos2.y;

	double xt2 = (-b2) / (2 * a2);
	double yt2 = m2 * xt2 + q2;

	//Construct obstacle
	Obstacle o;
	o.position = pos2;
	o.radius = r;
	o.T1.x = xt1;
	o.T1.y = yt1;
	o.T2.x = xt2;
	o.T2.y = yt2;
	return o;
}

// ======================
// Utility Functions
// ======================

bool v2_is_zero(vec2 v, double epsilon) {
    return fabs(v.x) < epsilon && fabs(v.y) < epsilon;
}