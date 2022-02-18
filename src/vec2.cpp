#include "vec2.h"
#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


double vec2::mod()const
{
  return sqrt(x * x + y * y);
}

vec2 vec2::rotate(double angle)const
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



vec2 vec2::norm()const
{
  vec2 n;
  double m = mod();
  n.x = x / m;
  n.y = y / m;
  return n;
}

vec2 vec2::add(const vec2& v2)const
{
  vec2 res;
  res.x = x + v2.x;
  res.y = y + v2.y;
  return res;
}

vec2 vec2::sub(const vec2& v2)const
{
  vec2 res;
  res.x = x - v2.x;
  res.y = y - v2.y;
  return res;
}

double vec2::dot(const vec2& v2)const
{
  return x * v2.x + y * v2.y;
}

vec2 vec2::addK(double k)const
{
  vec2 res;
  res.x = x + k;
  res.y = y + k;
  return res;
}

vec2 vec2::prodK(double k)const
{
  vec2 res;
  res.x = x * k;
  res.y = y * k;
  return res;
}

double vec2::distance(const vec2& v2)const
{
  vec2 res = sub(v2);
  return res.mod();
}

vec2 vec2::rotateHalfPI(int sign)const
{
  vec2 vr;
  vr.x = -sign * y;
  vr.y = sign * x;
  return vr;
}

vec2 vec2::rotate(const vec2& v)const
{
  vec2 dir = norm();

	vec2 dirp = sub(v).norm();

	double C = dir.dot(dirp);

	vec2 ret;

	if (C < -0.9999)
	{
		ret = {-x, -y};
	}
  else if (C >= -0.9999 && C < 0.9999)
	{
		ret = rotate(-acos(C));
		
	}
  else{
    ret = *this;
  }
  return ret;
}