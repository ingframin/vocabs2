#include "vec2.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

double v2_mod(vec2 v)
{
  return hypot(v.x,v.y);
}

vec2 v2_rotate(vec2 v, double angle)
{
  
  double m = hypot(v.x,v.y);
  vec2 ret;
  ret.x = v.x * cos(angle) - v.y * sin(angle);
  ret.y = v.x * sin(angle) + v.y * cos(angle);
  double retm = hypot(ret.x,ret.y);
  ret = v2_prodK(ret, m/retm);

  return ret;
}

vec2 v2_norm(vec2 v)
{
  vec2 n;
  double m = hypot(v.x,v.y);
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

vec2 v2_prodK(vec2 v, double k)
{
  vec2 res;
  res.x = v.x * k;
  res.y = v.y * k;
  return res;
}

double v2_distance(vec2 v1, vec2 v2)
{
  vec2 res = v2_sub(v1, v2);
  return hypot(res.x,res.y);
}
