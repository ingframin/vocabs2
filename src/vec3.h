#ifndef VEC3_H
#define VEC3_H

typedef struct
{
  double m00;
  double m01;
  double m02;
  double m10;
  double m11;
  double m12;
  double m20;
  double m21;
  double m22;
} mat3x3;


typedef struct vec3
{
    double x;
    double y;
    double z;
} vec3;

//module
double v3_mod(vec3 v);
//rotate
vec3 v3_rotateX(vec3 v, double angle);
vec3 v3_rotateY(vec3 v, double angle);
vec3 v3_rotateZ(vec3 v, double angle);

//normalize
vec3 v3_norm(vec3 v);
//Add and subtract
vec3 v3_add(vec3 v1, vec3 v2);
vec3 v3_sub(vec3 v1, vec3 v2);
//dot product
double v3_dot(vec3 v1, vec3 v2);
//add constant to both elements
vec3 v3_addK(vec3 v, double k);
//multiply both elements by k
vec3 v3_prodK(vec3 v, double k);
//distance between 2 vectors
double v3_distance(vec3 v1 , vec3 v2);

mat3x3 m33_rotation(double angleX, double angleY, double angleZ);
mat3x3 m33_product(mat3x3 m1, mat3x3 m2);
double m33_det(mat3x3 m);
vec3 v3_cross(vec3 v1, vec3 v2);
#endif;