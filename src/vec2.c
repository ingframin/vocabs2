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

#include "vec2.h"
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif
#include <stdlib.h>
#include <stdbool.h>

// Helper function to check if a vector is approximately zero
static bool v2_is_zero(vec2 v, double epsilon) {
    return fabs(v.x) < epsilon && fabs(v.y) < epsilon;
}

double v2_mod(vec2 v)
{
  return hypot(v.x, v.y);
}

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
  
  double C = cos(angle);
  double S = sin(angle);

  if(v2_is_zero(v, 1e-12)) {
    // Cannot rotate zero vector
    return v;
  }
  
  double m = v2_mod(v);
  vec2 vn;
  vn.x = v.x / m;
  vn.y = v.y / m;
  vec2 ret;

  ret.x = m * (vn.x * C - vn.y * S);
  ret.y = m * (vn.x * S + vn.y * C);

  return ret;
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

double v2_dot(vec2 v1, vec2 v2)
{
  return v1.x * v2.x + v1.y * v2.y;
}

vec2 v2_addK(vec2 v, double k)
{
  vec2 res;
  res.x = v.x + k;
  res.y = v.y + k;
  return res;
}

vec2 v2_scale(vec2 v, double k)
{
  vec2 res;
  res.x = v.x * k;
  res.y = v.y * k;
  return res;
}

double v2_distance(vec2 v1, vec2 v2)
{
  vec2 res = v2_diff(v1, v2);
  return v2_mod(res);
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

vec2 v2_lerp(vec2 p1, vec2 p2, double t){
  vec2 a = v2_scale(p1,1.0-t);
  vec2 b = v2_scale(p2,t);
  return v2_add(a,b);

}

vec2 v2_qspline(vec2 p1, vec2 p2, vec2 p3, double t){
    vec2 p0i = v2_lerp(p1,p2,t);
    vec2 p1i = v2_lerp(p2,p3,t);
    
    return v2_lerp(p0i,p1i,t);
}

vec2 v2_cspline(vec2 p1, vec2 p2, vec2 p3, vec2 p4, double t){
    vec2 p0i = v2_lerp(p1,p2,t);
    vec2 p1i = v2_lerp(p2,p3,t);
    vec2 p2i = v2_lerp(p3,p4,t);
    
    return v2_qspline(p0i,p1i,p2i,t);
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
    // This is more efficient than the original nested loop approach
    for (size_t level = 1; level < vs_len; ++level) {
        for (size_t i = 0; i < vs_len - level; ++i) {
            temp[i] = v2_lerp(temp[i], temp[i + 1], t);
        }
    }

    // The result is in temp[0], but we return the whole array
    // as documented (caller must free)
    return temp;
}

// Add a function to free the interpolated array
void v2_free_interpolated(vec2* points) {
    if (points) {
        free(points);
    }
}