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

#ifndef MATH2D_H
#define MATH2D_H

#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <immintrin.h>

// 2D Vector class
class vec2 {
public:
    double x;
    double y;
    
    // Constructors
    vec2() : x(0.0), y(0.0) {}
    vec2(double x, double y) : x(x), y(y) {}
    
    // Basic operations
    double mod() const;
    vec2 normalize() const;
    
    // Vector operations
    double distanceTo(const vec2& other) const;
    double angleTo(const vec2& other) const;
    double dot(const vec2& other) const;
    
    // Transformations
    vec2 rotate(double angle) const;
    vec2 rotateLeftHalfPI() const;
    vec2 rotateRightHalfPI() const;
    vec2 reverse() const;
    
    // Operator overloading
    vec2 operator+(const vec2& other) const;
    vec2 operator-(const vec2& other) const;
    vec2 operator*(double k) const;
    vec2 operator+(double k) const;
    
    bool operator==(const vec2& other) const;
    bool operator!=(const vec2& other) const;
    
    // Utility
    bool isZero(double epsilon = 1e-12) const;
};

// Barycentric coordinates class
class barycoords {
public:
    double alpha;
    double beta;
    double gamma;
    
    barycoords() : alpha(0.0), beta(0.0), gamma(0.0) {}
    barycoords(double alpha, double beta, double gamma) : alpha(alpha), beta(beta), gamma(gamma) {}
};

// Obstacle class (for velocity obstacle calculations)
class Obstacle {
public:
    //radius
    double radius;
    //center
    vec2 position;
    //tangent points
    vec2 T1;
    vec2 T2;
    
    Obstacle() : radius(0.0) {}
    Obstacle(double radius, vec2 position, vec2 T1, vec2 T2) 
        : radius(radius), position(position), T1(T1), T2(T2) {}
};

// ======================
// Vector Operations (free functions)
// ======================

double v2_dot(const vec2& v1, const vec2& v2);

// ======================
// Geometric Transformations
// ======================

vec2 v2_rotate(const vec2& v, double angle);
vec2 v2_rotateLeftHalfPI(const vec2& v);
vec2 v2_rotateRightHalfPI(const vec2& v);
vec2 v2_reverse(const vec2& v);

// ======================
// Distance and Angle Calculations
// ======================

double v2_distance(const vec2& v1, const vec2& v2);
double v2_angle_between(const vec2& v1, const vec2& v2);

// ======================
// Interpolation
// ======================

vec2 v2_lerp(const vec2& p1, const vec2& p2, double t);
vec2 v2_qspline(const vec2& p1, const vec2& p2, const vec2& p3, double t);
vec2 v2_cspline(const vec2& p1, const vec2& p2, const vec2& p3, const vec2& p4, double t);
vec2* v2_interpolate(const vec2 vs[], size_t vs_len, double t);
void v2_free_interpolated(vec2* points);

// ======================
// Barycentric Coordinates
// ======================

barycoords v2_barycentric(const vec2& A, const vec2& B, const vec2& C, const vec2& P);

// ======================
// Obstacle Calculations (Velocity Obstacle Method)
// ======================

Obstacle compute_obstacle(const vec2& pos1, const vec2& pos2, double size1, double size2);

#endif