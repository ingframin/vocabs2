#include "vec3.h"
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif
#include <stdlib.h>

double v3_mod(vec3 v)
{
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

vec3 v3_rotateX(vec3 v, double angle){
    double m = v3_mod(v);
    vec3 vn;
    // vn.x = v.x / m;
    vn.y = v.y / m;
    vn.z = v.z / m;
    double C = cos(angle);
    double S = sin(angle);
    vec3 ret;
    ret.y = m * (vn.y * C - vn.z * S);
    ret.z = m * (vn.y * S + vn.z * C);
    ret.x = v.x;
    return ret;
}
vec3 v3_rotateY(vec3 v, double angle){
    double m = v3_mod(v);
    vec3 vn;
    vn.x = v.x / m;
    //vn.y = v.y / m;
    vn.z = v.z / m;
    double C = cos(angle);
    double S = sin(angle);
    vec3 ret;
    ret.x = m * (vn.x * C - vn.z * S);
    ret.z = m * (vn.x * S + vn.z * C);
    ret.y = v.y;
    return ret;

}

vec3 v3_rotateZ(vec3 v, double angle){
    double m = v3_mod(v);
    vec3 vn;
    vn.x = v.x / m;
    vn.y = v.y / m;
    //vn.z = v.z / m;
    double C = cos(angle);
    double S = sin(angle);
    vec3 ret;
    ret.x = m * (vn.x * C - vn.y * S);
    ret.y = m * (vn.x * S + vn.y * C);
    ret.z = v.z;
    return ret;
}


mat3x3 m33_product(mat3x3 m1, mat3x3 m2){
    mat3x3 res;
    res.m00 = m1.m00*m2.m00 + m1.m01*m2.m10 + m1.m02*m2.m20;
    res.m01 = m1.m00*m2.m01 + m1.m01*m2.m11 + m1.m02*m2.m21;
    res.m02 = m1.m00*m2.m02 + m1.m01*m2.m12 + m1.m02*m2.m22;

    res.m10 = m1.m10*m2.m00 + m1.m11*m2.m10 + m1.m12*m2.m20;
    res.m11 = m1.m10*m2.m01 + m1.m11*m2.m11 + m1.m12*m2.m21;
    res.m12 = m1.m10*m2.m02 + m1.m11*m2.m12 + m1.m12*m2.m22;

    res.m20 = m1.m20*m2.m00 + m1.m21*m2.m10 + m1.m22*m2.m20;
    res.m21 = m1.m20*m2.m01 + m1.m21*m2.m11 + m1.m22*m2.m21;
    res.m22 = m1.m20*m2.m02 + m1.m21*m2.m12 + m1.m22*m2.m22;
    return res;
}
mat3x3 m33_rotation(double angleX, double angleY, double angleZ){
    mat3x3 Rz;
    mat3x3 Ry;
    mat3x3 Rx;

    Rz.m00 = cos(angleZ);
    Rz.m01 = -sin(angleZ);
    Rz.m02 = 0;

    Rz.m10 = sin(angleZ);
    Rz.m11 = cos(angleZ);
    Rz.m12 = 0;

    Rz.m10 = 0;
    Rz.m11 = 0;
    Rz.m12 = 1;

    Ry.m00 = cos(angleY);
    Ry.m01 = 0;
    Ry.m02 = sin(angleY);

    Ry.m10 = 0;
    Ry.m11 = 1;
    Ry.m12 = 0;

    Ry.m10 = -sin(angleY);
    Ry.m11 = 0;
    Ry.m12 = cos(angleY);

    
    Rx.m00 = 1;
    Rx.m01 = 0;
    Rx.m02 = 0;

    Rx.m10 = 0;
    Rx.m11 = cos(angleX);
    Rx.m12 = -sin(angleX);

    Rx.m10 = 0;
    Rx.m11 = sin(angleX);
    Rx.m12 = cos(angleX);

    mat3x3 R1 = m33_product(Rz,Ry);
    return m33_product(R1,Rx);
    

}

vec3 v3_norm(vec3 v)
{
  vec3 n;
  double m = v3_mod(v);
  n.x = v.x / m;
  n.y = v.y / m;
  n.z = v.z / m;
  return n;
}

vec3 v3_add(vec3 v1, vec3 v2)
{
  vec3 res;
  res.x = v1.x + v2.x;
  res.y = v1.y + v2.y;
  res.z = v1.z + v2.z;
  return res;
}

vec3 v3_sub(vec3 v1, vec3 v2)
{
  vec3 res;
  res.x = v1.x - v2.x;
  res.y = v1.y - v2.y;
  res.z = v1.z - v2.z;
  return res;
}

double v3_dot(vec3 v1, vec3 v2)
{
  return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

vec3 v3_addK(vec3 v, double k)
{
  vec3 res;
  res.x = v.x + k;
  res.y = v.y + k;
  res.z = v.z + k;
  return res;
}

vec3 v3_prodK(vec3 v, double k)
{
  vec3 res;
  res.x = v.x * k;
  res.y = v.y * k;
  res.z = v.z * k;
  return res;
}

double v3_distance(vec3 v1, vec3 v2)
{
  vec3 res = v3_sub(v1, v2);
  return v3_mod(res);
}
double m33_det(mat3x3 m){
    double g1 = m.m00*m.m11*m.m22-m.m22*m.m10*m.m01;
    double g2 = m.m01*m.m12*m.m20-m.m20*m.m11*m.m02;
    double g3 = m.m02*m.m10*m.m21-m.m21*m.m12*m.m00;
    return g1+g2+g3;
}

vec3 v3_cross(vec3 v1, vec3 v2){
    vec3 res;
    res.x = v1.y*v2.z - v2.y*v1.z;
    res.y = v1.x*v2.z - v2.x*v1.z;
    res.z = v1.y*v2.x - v2.y*v1.x;
    return res;
    
}