#include "vec2.h"
#include <cmath>
#include <numbers>

double vec2::mod()
{
  return sqrt(x * x + y * y);
}

vec2 vec2::rotate(double angle)
{

  double m = mod();
  vec2 vn;
  vn.x = x / m;
  vn.y = y / m;
  vec2 ret;
  double C = cos(angle);
  double S = sin(angle);

  ret.x = m * (vn.x * C - vn.y * S);
  ret.y = m * (vn.x * S + vn.y * C);

  return ret;
}

vec2 vec2::norm()
{
  vec2 n;
  double m = mod();
  n.x = x / m;
  n.y = y / m;
  return n;
}

vec2 vec2::add(vec2& v2)
{
  vec2 res;
  res.x = x + v2.x;
  res.y = y + v2.y;
  return res;
}

vec2 vec2::sub(vec2& v2)
{
  vec2 res;
  res.x = x - v2.x;
  res.y = y - v2.y;
  return res;
}

double vec2::dot(vec2& v2)
{
  return x * v2.x + y * v2.y;
}

vec2 vec2::addK(double k)
{
  vec2 res;
  res.x = x + k;
  res.y = y + k;
  return res;
}

vec2 vec2::prodK(double k)
{
  vec2 res;
  res.x = x * k;
  res.y = y * k;
  return res;
}

double vec2::distance(vec2& v2)
{
  vec2 res = sub(v2);
  return res.mod();
}

vec2 vec2::rotateHalfPI(int sign)
{
  vec2 vr;
  vr.x = -sign * y;
  vr.y = sign * x;
  return vr;
}