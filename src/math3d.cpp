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

#include "math3d.h"
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif
#include <xmmintrin.h>
#include <immintrin.h>

// ======================
// vec3 Class Methods
// ======================

double vec3::mod() const
{
    return sqrt(x * x + y * y + z * z);
}

vec3 vec3::normalize() const
{
    double m = mod();
    if (m < 1e-12) {
        return vec3(0.0, 0.0, 0.0);
    }
    return vec3(x / m, y / m, z / m);
}

vec3 vec3::operator+(const vec3& other) const
{
    return vec3(x + other.x, y + other.y, z + other.z);
}

vec3 vec3::operator-(const vec3& other) const
{
    return vec3(x - other.x, y - other.y, z - other.z);
}

vec3 vec3::operator*(double k) const
{
    return vec3(x * k, y * k, z * k);
}

bool vec3::operator==(const vec3& other) const
{
    return x == other.x && y == other.y && z == other.z;
}

bool vec3::operator!=(const vec3& other) const
{
    return !(*this == other);
}

// ======================
// mat3x3 Class Methods
// ======================

mat3x3::mat3x3() {
    x1 = y1 = z1 = 0.0;
    x2 = y2 = z2 = 0.0;
    x3 = y3 = z3 = 0.0;
}

mat3x3::mat3x3(double x1, double y1, double z1,
               double x2, double y2, double z2,
               double x3, double y3, double z3)
    : x1(x1), y1(y1), z1(z1),
      x2(x2), y2(y2), z2(z2),
      x3(x3), y3(y3), z3(z3) {}

// ======================
// mat4x4 Class Methods
// ======================

mat4x4::mat4x4() {
    x1 = y1 = z1 = w1 = 0.0;
    x2 = y2 = z2 = w2 = 0.0;
    x3 = y3 = z3 = w3 = 0.0;
}

mat4x4::mat4x4(double x1, double y1, double z1, double w1,
               double x2, double y2, double z2, double w2,
               double x3, double y3, double z3, double w3)
    : x1(x1), y1(y1), z1(z1), w1(w1),
      x2(x2), y2(y2), z2(z2), w2(w2),
      x3(x3), y3(y3), z3(z3), w3(w3) {}

// ======================
// Construction functions
// ======================

Cone newCone(vec3 Sc, double radius, vec3 P){
    Cone c;
    c.vertex = P;
    vec3 dif = v3_diff(P, Sc);
    double d = v3_mod(dif);
    double l = sqrt(d * d - radius * radius);
    double half_aperture = asin(radius / d);
    c.aperture = 2 * half_aperture;
    double h = l * cos(half_aperture);
    vec3 cp = v3_diff(Sc, P);
    c.axis = v3_scale(cp, h);
    return c;
}

// ======================
// Vector operations
// ======================

vec3 v3_add(const vec3& v1, const vec3& v2){
    return vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

vec3 v3_diff(const vec3& v1, const vec3& v2){
    return vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

vec3 v3_normalize(const vec3& v1){
    return v1.normalize();
}

double v3_mod(const vec3& v1){
    return v1.mod();
}

double v3_distance(const vec3& v1, const vec3& v2){
    vec3 dif = v3_diff(v1, v2);
    return dif.mod();
}

double v3_angle_between(const vec3& v1, const vec3& v2){
    double dot = v3_dot(v1, v2);
    double mod1 = v1.mod();
    double mod2 = v2.mod();
    
    if (mod1 < 1e-12 || mod2 < 1e-12) {
        return 0.0;
    }
    
    double cos_theta = dot / (mod1 * mod2);
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;
    
    return acos(cos_theta);
}

double v3_dot(const vec3& v1, const vec3& v2){
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

vec3 v3_cross(const vec3& v1, const vec3& v2){
    return vec3(
        v1.y * v2.z - v1.z * v2.y,
        v1.z * v2.x - v1.x * v2.z,
        v1.x * v2.y - v1.y * v2.x
    );
}

vec3 v3_scale(const vec3& v, double k){
    return vec3(v.x * k, v.y * k, v.z * k);
}

// ======================
// SIMD versions
// ======================

double v3_dot_simd(const vec3& v1, const vec3& v2){
    __m128d a = _mm_set_pd(v1.y, v1.x);
    __m128d b = _mm_set_pd(v2.y, v2.x);
    __m128d c = _mm_set_pd(v1.z, 0.0);
    __m128d d = _mm_set_pd(v2.z, 0.0);
    
    __m128d ab = _mm_mul_pd(a, b);
    __m128d cd = _mm_mul_pd(c, d);
    __m128d abcd = _mm_add_pd(ab, cd);
    
    double result[2];
    _mm_storeu_pd(result, abcd);
    
    return result[0] + result[1];
}

// ======================
// Matrix operations
// ======================

mat3x3 simd_m33_product_m33(const mat3x3& m1, const mat3x3& m2){
    mat3x3 result;
    
    // Row 1
    result.x1 = m1.x1 * m2.x1 + m1.y1 * m2.x2 + m1.z1 * m2.x3;
    result.y1 = m1.x1 * m2.y1 + m1.y1 * m2.y2 + m1.z1 * m2.y3;
    result.z1 = m1.x1 * m2.z1 + m1.y1 * m2.z2 + m1.z1 * m2.z3;
    
    // Row 2
    result.x2 = m1.x2 * m2.x1 + m1.y2 * m2.x2 + m1.z2 * m2.x3;
    result.y2 = m1.x2 * m2.y1 + m1.y2 * m2.y2 + m1.z2 * m2.y3;
    result.z2 = m1.x2 * m2.z1 + m1.y2 * m2.z2 + m1.z2 * m2.z3;
    
    // Row 3
    result.x3 = m1.x3 * m2.x1 + m1.y3 * m2.x2 + m1.z3 * m2.x3;
    result.y3 = m1.x3 * m2.y1 + m1.y3 * m2.y2 + m1.z3 * m2.y3;
    result.z3 = m1.x3 * m2.z1 + m1.y3 * m2.z2 + m1.z3 * m2.z3;
    
    return result;
}

mat3x3 m33_product_m33(const mat3x3& m1, const mat3x3& m2){
    return simd_m33_product_m33(m1, m2);
}

vec3 m33_product_v3(const mat3x3& m1, const vec3& v2){
    return vec3(
        m1.x1 * v2.x + m1.y1 * v2.y + m1.z1 * v2.z,
        m1.x2 * v2.x + m1.y2 * v2.y + m1.z2 * v2.z,
        m1.x3 * v2.x + m1.y3 * v2.y + m1.z3 * v2.z
    );
}

mat3x3 m33_transpose(const mat3x3& dP){
    return mat3x3(
        dP.x1, dP.x2, dP.x3,
        dP.y1, dP.y2, dP.y3,
        dP.z1, dP.z2, dP.z3
    );
}

// ======================
// Geometric transformations
// ======================

mat3x3 m33_rotationX(double angle){
    double cos_theta = cos(angle);
    double sin_theta = sin(angle);
    
    return mat3x3(
        1.0, 0.0, 0.0,
        0.0, cos_theta, -sin_theta,
        0.0, sin_theta, cos_theta
    );
}

mat3x3 m33_rotationY(double angle){
    double cos_theta = cos(angle);
    double sin_theta = sin(angle);
    
    return mat3x3(
        cos_theta, 0.0, sin_theta,
        0.0, 1.0, 0.0,
        -sin_theta, 0.0, cos_theta
    );
}

mat3x3 m33_rotationZ(double angle){
    double cos_theta = cos(angle);
    double sin_theta = sin(angle);
    
    return mat3x3(
        cos_theta, -sin_theta, 0.0,
        sin_theta, cos_theta, 0.0,
        0.0, 0.0, 1.0
    );
}

mat3x3 m33_rotation_XYZ(double alpha, double beta, double gamma){
    mat3x3 rx = m33_rotationX(alpha);
    mat3x3 ry = m33_rotationY(beta);
    mat3x3 rz = m33_rotationZ(gamma);
    
    // R = Rz * Ry * Rx
    mat3x3 temp = m33_product_m33(ry, rx);
    return m33_product_m33(rz, temp);
}

mat3x3 m33_scale_xyz_v3(const vec3& sxyz){
    return mat3x3(
        sxyz.x, 0.0, 0.0,
        0.0, sxyz.y, 0.0,
        0.0, 0.0, sxyz.z
    );
}

mat3x3 m33_scale_xyz(double k){
    return mat3x3(
        k, 0.0, 0.0,
        0.0, k, 0.0,
        0.0, 0.0, k
    );
}

// ======================
// Angle calculations
// ======================

double v3_angle_to_x(const vec3& v){
    return atan2(sqrt(v.y * v.y + v.z * v.z), v.x);
}

double v3_angle_to_y(const vec3& v){
    return atan2(sqrt(v.x * v.x + v.z * v.z), v.y);
}

double v3_angle_to_z(const vec3& v){
    return atan2(sqrt(v.x * v.x + v.y * v.y), v.z);
}

// ======================
// Barycentric coordinates
// ======================

vec3 v3_barycentric(const vec3& P, const vec3& A, const vec3& B, const vec3& C){
    vec3 v0 = v3_diff(B, A);
    vec3 v1 = v3_diff(C, A);
    vec3 v2 = v3_diff(P, A);
    
    double dot00 = v3_dot(v0, v0);
    double dot01 = v3_dot(v0, v1);
    double dot02 = v3_dot(v0, v2);
    double dot11 = v3_dot(v1, v1);
    double dot12 = v3_dot(v1, v2);
    
    double inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    double beta = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    double gamma = (dot00 * dot12 - dot01 * dot02) * inv_denom;
    double alpha = 1.0 - beta - gamma;
    
    return vec3(alpha, beta, gamma);
}

// ======================
// Cone operations
// ======================

bool is_point_within_cone(const vec3& P, const Cone& C){
    vec3 CP = v3_diff(P, C.vertex);
    double CP_mod = CP.mod();
    
    if (CP_mod < 1e-12) {
        return true; // Point is at vertex
    }
    
    vec3 Caxis_norm = C.axis.normalize();
    vec3 CP_norm = CP.normalize();
    
    double angle = v3_angle_between(CP_norm, Caxis_norm);
    
    return (angle <= C.aperture / 2.0);
}

// ======================
// Interpolation
// ======================

vec3 v3_lerp(const vec3& p1, const vec3& p2, double t){
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    
    return vec3(
        p1.x + t * (p2.x - p1.x),
        p1.y + t * (p2.y - p1.y),
        p1.z + t * (p2.z - p1.z)
    );
}