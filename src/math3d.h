
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

#ifndef MATH3D
#define MATH3D
#include <stdbool.h>

typedef struct vec3
{
    double x;
    double y;
    double z;
    
} vec3;

typedef struct mat3x3{
    double x1;
    double y1;
    double z1;

    double x2;
    double y2;
    double z2;

    double x3;
    double y3;
    double z3;

} mat3x3;


typedef struct Cone{
    //vertex
    vec3 vertex;
    //oriented height
    vec3 axis;
    //aperture angle
    double aperture;
   
} Cone;


Cone newCone(vec3 Sc, double radius, vec3 P);

vec3 v3_add(vec3 v1, vec3 v2);

vec3 v3_diff(vec3 v1, vec3 v2);

vec3 v3_normalize(vec3 v1);

double v3_mod(vec3 v1);
double v3_distance(vec3 v1, vec3 v2);
double v3_angle_between(vec3 v1, vec3 v2);

double v3_dot(vec3 v1, vec3 v2);
vec3 v3_cross(vec3 v1, vec3 v2);
vec3 v3_scale(vec3 v, double k);
mat3x3 m33_product_m33(mat3x3 m1, mat3x3 m2);
vec3 m33_product_v3(mat3x3 m1, vec3 v2);

mat3x3 m33_transpose(mat3x3 dP);

// Geometric transformations
mat3x3 m33_rotationX(double angle);
mat3x3 m33_rotationY(double angle);
mat3x3 m33_rotationZ(double angle);

mat3x3 m33_rotation_XYZ(double alpha, double beta, double gamma);

mat3x3 m33_scale_xyz_v3(vec3 sxyz);
mat3x3 m33_scale_xyz(double k);

double v3_angle_to_x(vec3 v);
double v3_angle_to_y(vec3 v);
double v3_angle_to_z(vec3 v);

vec3 v3_barycentric(vec3 P, vec3 A, vec3 B, vec3 C);
bool is_point_within_cone(vec3 P, Cone C);
//linear interpolation between 2 points
vec3 v3_lerp(vec3 p1, vec3 p2, double t);


#endif