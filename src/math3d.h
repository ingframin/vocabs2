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

#include <cmath>

class vec3 {
public:
    double x;
    double y;
    double z;
    
    vec3() : x(0.0), y(0.0), z(0.0) {}
    vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    
    // Basic operations
    double mod() const;
    vec3 normalize() const;
    
    // Vector operations
    double distanceTo(const vec3& other) const;
    double angleTo(const vec3& other) const;
    double dot(const vec3& other) const;
    vec3 cross(const vec3& other) const;
    
    // Angle calculations
    double angleToX() const;
    double angleToY() const;
    double angleToZ() const;
    
    // Operator overloading
    vec3 operator+(const vec3& other) const;
    vec3 operator-(const vec3& other) const;
    vec3 operator*(double k) const;
    
    bool operator==(const vec3& other) const;
    bool operator!=(const vec3& other) const;
};

class vec4 {
public:
    double x;
    double y;
    double z;
    double w;
    
    vec4() : x(0.0), y(0.0), z(0.0), w(0.0) {}
    vec4(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
};

class mat3x3 {
public:
    double x1, y1, z1;
    double x2, y2, z2;
    double x3, y3, z3;
    
    mat3x3();
    mat3x3(double x1, double y1, double z1,
           double x2, double y2, double z2,
           double x3, double y3, double z3);
};

class mat4x4 {
public:
    double x1, y1, z1, w1;
    double x2, y2, z2, w2;
    double x3, y3, z3, w3;
    
    mat4x4();
    mat4x4(double x1, double y1, double z1, double w1,
           double x2, double y2, double z2, double w2,
           double x3, double y3, double z3, double w3);
};

class Cone {
public:
    vec3 vertex;    // vertex
    vec3 axis;      // oriented height
    double aperture; // aperture angle
    
    Cone() : aperture(0.0) {}
    Cone(vec3 vertex, vec3 axis, double aperture) 
        : vertex(vertex), axis(axis), aperture(aperture) {}
};

// Construction functions
Cone newCone(vec3 Sc, double radius, vec3 P);

// Vector operations
vec3 v3_add(const vec3& v1, const vec3& v2);
vec3 v3_diff(const vec3& v1, const vec3& v2);
vec3 v3_normalize(const vec3& v1);
double v3_mod(const vec3& v1);
double v3_distance(const vec3& v1, const vec3& v2);
double v3_angle_between(const vec3& v1, const vec3& v2);
double v3_dot(const vec3& v1, const vec3& v2);
vec3 v3_cross(const vec3& v1, const vec3& v2);
vec3 v3_scale(const vec3& v, double k);

// SIMD versions
double v3_dot_simd(const vec3& v1, const vec3& v2);

// Matrix operations
mat3x3 simd_m33_product_m33(const mat3x3& m1, const mat3x3& m2);
mat3x3 m33_product_m33(const mat3x3& m1, const mat3x3& m2);
vec3 m33_product_v3(const mat3x3& m1, const vec3& v2);
mat3x3 m33_transpose(const mat3x3& dP);

// Geometric transformations
mat3x3 m33_rotationX(double angle);
mat3x3 m33_rotationY(double angle);
mat3x3 m33_rotationZ(double angle);
mat3x3 m33_rotation_XYZ(double alpha, double beta, double gamma);
mat3x3 m33_scale_xyz_v3(const vec3& sxyz);
mat3x3 m33_scale_xyz(double k);

// Angle calculations
double v3_angle_to_x(const vec3& v);
double v3_angle_to_y(const vec3& v);
double v3_angle_to_z(const vec3& v);

// Barycentric coordinates
vec3 v3_barycentric(const vec3& P, const vec3& A, const vec3& B, const vec3& C);

// Cone operations
bool is_point_within_cone(const vec3& P, const Cone& C);

// Interpolation
vec3 v3_lerp(const vec3& p1, const vec3& p2, double t);

#endif