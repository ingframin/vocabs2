
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

double v2_mod(vec2 v)
{
  return hypot(v.x, v.y);
}

vec2 v2_rotate(vec2 v, double angle)
{
  double C = cos(angle);
  double S = sin(angle);

  if(fabs(angle) < 1e-4){
     return v;
  }
  else if(fabs(angle - M_PI) < 1e-4){
    return v2_rotatePI(v);
  }
  if(fabs(angle - M_PI/2) < 1e-4){
    return v2_rotateLeftHalfPI(v);
  }
  else if(fabs(angle + M_PI/2) < 1e-4){
    return v2_rotatePI(v2_rotateLeftHalfPI(v));
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
  vec2 n;
  double m = v2_mod(v);
  n.x = v.x / m;
  n.y = v.y / m;
  return n;
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

vec2 v2_rotatePI(vec2 p){
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
  double mods = v2_mod(v1)*v2_mod(v2);
  double sina = (v1.x*v2.y - v2.x*v1.y)/mods;
  double cosa = (v1.x*v2.x + v1.y*v2.y)/mods;

  return atan2(sina, cosa);

}

vec2 v2_interpolate(vec2 vs[], size_t vs_len, double t){
  if(vs_len == 1){
      return vs[0];
  }
  
  for(size_t L = vs_len-1; L>1; L--){
    
    for(size_t i = 1; i < L; i++){
      vs[i-1] = v2_lerp(vs[i],vs[i-1],t);
    }

  }
  vec2 res = vs[0];
  
  return res; 
}