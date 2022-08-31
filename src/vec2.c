#include "vec2.h"
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif
#include <stdlib.h>

double v2_mod(vec2 v)
{
  return sqrt(v.x * v.x + v.y * v.y);
}

vec2 v2_rotate(vec2 v, double angle)
{

  double m = v2_mod(v);
  vec2 vn;
  vn.x = v.x / m;
  vn.y = v.y / m;
  vec2 ret;
  double C = cos(angle);
  double S = sin(angle);

  ret.x = m * (vn.x * C - vn.y * S);
  ret.y = m * (vn.x * S + vn.y * C);

  return ret;
}

vec2 v2_norm(vec2 v)
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

vec2 v2_sub(vec2 v1, vec2 v2)
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
  vec2 res = v2_sub(v1, v2);
  return v2_mod(res);
}

vec2 v2_rotateHalfPI(vec2 v, int sign)
{
  vec2 vr;
  vr.x = -sign * v.y;
  vr.y = sign * v.x;
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