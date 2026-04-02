
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
#include <stdint.h>
#include <stdbool.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif
#include <xmmintrin.h>
#include <immintrin.h>



// Refactored code for improved performance and efficiency
Cone newCone(vec3 Sc, double radius, vec3 P){
    Cone c;
    c.vertex = P;
    vec3 dif = v3_diff(P,Sc);
    double d = v3_mod(dif);
    double l = sqrt(d * d - radius * radius);
    double half_aperture = asin(radius / d);
    c.aperture = 2 * half_aperture;
    double h = l * cos(half_aperture);
    vec3 cp = v3_diff(Sc, P);
    c.axis = v3_scale(cp, h);
    return c;
}



vec3 v3_add(vec3 v1, vec3 v2){
    
    return (vec3){v1.x+v2.x,v1.y+v2.y,v1.z+v2.z};	
}

vec3 v3_diff(vec3 v1, vec3 v2){

    return (vec3){v1.x-v2.x,v1.y-v2.y,v1.z-v2.z};
}

double v3_dot(vec3 v1, vec3 v2){
    return v1.x*v2.x+v1.y*v2.y+v1.z*v2.z;
}
// Function to compute the dot product of two 3D vectors using SIMD
double v3_dot_simd(vec3 v1, vec3 v2) {
    // Load the vector components into SIMD registers
    __m128d v1_vals = _mm_set_pd(v1.z, v1.y);
    __m128d v2_vals = _mm_set_pd(v2.z, v2.y);
    __m128d v1_x = _mm_set_pd(0.0, v1.x);
    __m128d v2_x = _mm_set_pd(0.0, v2.x);

    // Perform multiplication and accumulate results
    __m128d product1 = _mm_mul_pd(v1_vals, v2_vals);
    __m128d product2 = _mm_mul_pd(v1_x, v2_x);

    // Horizontal addition to sum all components
    __m128d sum1 = _mm_hadd_pd(product1, product2);
    __m128d sum2 = _mm_hadd_pd(sum1, sum1);
    double result[2];
    _mm_storeu_pd(result, sum2);
    // The result is in both positions of the array due to _mm_hadd_pd
    return result[0];
}

// Function to compute the magnitude of a 3D vector using SIMD
double v3_mod(vec3 v1) {
    double dot_result = v3_dot_simd(v1, v1);
    return sqrt(dot_result);
}

// Function to compute the Euclidean distance between two 3D vectors using SIMD
double v3_distance(vec3 v1, vec3 v2) {
    // Compute differences in all components at once
    __m128d dx = _mm_set_pd(v1.z - v2.z, v1.x - v2.x);
    __m128d dy = _mm_set_pd(0.0, v1.y - v2.y);

    // Square the differences
    __m128d dx_sqr = _mm_mul_pd(dx, dx);
    __m128d dy_sqr = _mm_mul_pd(dy, dy);

    // Sum all squared differences
    __m128d sum = _mm_add_pd(dx_sqr, dy_sqr);
    sum = _mm_hadd_pd(sum, sum);

    // Extract and return square root
    return sqrt(_mm_cvtsd_f64(sum));
}

double v3_angle_between(vec3 v1, vec3 v2){
    double sqmag1 = v1.x*v1.x + v1.y*v1.y + v1.z*v1.z;
    double sqmag2 = v2.x*v2.x + v2.y*v2.y + v2.z*v2.z;
    double dp = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
    return acos(fmin(fmax(dp/(sqrt(sqmag1)*sqrt(sqmag2)), -1.0), 1.0));

}

vec3 v3_angle_to_axis(vec3 v){
    double modv = v3_mod(v);
    double inv_modv = 1.0 / modv;

    double cx = v.x * inv_modv;
    double cy = v.y * inv_modv;
    double cz = v.z * inv_modv;

    // Clamp values to avoid NaN when acos is used
    cx = fmin(fmax(cx, -1.0), 1.0);
    cy = fmin(fmax(cy, -1.0), 1.0);
    cz = fmin(fmax(cz, -1.0), 1.0);

    return (vec3){acos(cx),acos(cy),acos(cz)};
}

vec3 v3_cross(vec3 v1, vec3 v2){

    return (vec3){v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x};
}

vec3 v3_normalize(vec3 v1){
    double mag = v3_mod(v1);
    if(mag == 0) return (vec3){0,0,0};
    return (vec3){v1.x/mag, v1.y/mag, v1.z/mag};
}

vec3 v3_scale(vec3 v, double k){
    vec3 sv = {v.x*k, v.y*k, v.z*k}; 
    return sv;
}


// Fixed SIMD matrix multiplication function for mat3x3 structures
// Note: Current mat3x3 struct doesn't store rows contiguously, so we use
// a different approach that works with the existing layout
mat3x3 simd_m33_product_m33(mat3x3 m1, mat3x3 m2) {
    mat3x3 result;

    // Compute each element of the result matrix individually using SIMD
    // First row
    __m128d m1_x1 = _mm_set_pd(m1.z1, m1.y1);
    __m128d m1_x1_x = _mm_set_pd(0.0, m1.x1);
    __m128d m2_col1 = _mm_set_pd(m2.z1, m2.y1);
    __m128d m2_col1_x = _mm_set_pd(0.0, m2.x1);
    
    __m128d product1 = _mm_mul_pd(m1_x1, m2_col1);
    __m128d product2 = _mm_mul_pd(m1_x1_x, m2_col1_x);
    __m128d sum1 = _mm_add_pd(product1, product2);
    __m128d sum2 = _mm_hadd_pd(sum1, sum1);
    double temp[2];
    _mm_storeu_pd(temp, sum2);
    result.x1 = temp[0];

    // result.y1
    __m128d m2_col2 = _mm_set_pd(m2.z2, m2.y2);
    __m128d m2_col2_x = _mm_set_pd(0.0, m2.x2);
    product1 = _mm_mul_pd(m1_x1, m2_col2);
    product2 = _mm_mul_pd(m1_x1_x, m2_col2_x);
    sum1 = _mm_add_pd(product1, product2);
    sum2 = _mm_hadd_pd(sum1, sum1);
    _mm_storeu_pd(temp, sum2);
    result.y1 = temp[0];

    // result.z1
    __m128d m2_col3 = _mm_set_pd(m2.z3, m2.y3);
    __m128d m2_col3_x = _mm_set_pd(0.0, m2.x3);
    product1 = _mm_mul_pd(m1_x1, m2_col3);
    product2 = _mm_mul_pd(m1_x1_x, m2_col3_x);
    sum1 = _mm_add_pd(product1, product2);
    sum2 = _mm_hadd_pd(sum1, sum1);
    _mm_storeu_pd(temp, sum2);
    result.z1 = temp[0];

    // Second row
    __m128d m1_x2 = _mm_set_pd(m1.z2, m1.y2);
    __m128d m1_x2_x = _mm_set_pd(0.0, m1.x2);
    
    product1 = _mm_mul_pd(m1_x2, m2_col1);
    product2 = _mm_mul_pd(m1_x2_x, m2_col1_x);
    sum1 = _mm_add_pd(product1, product2);
    sum2 = _mm_hadd_pd(sum1, sum1);
    _mm_storeu_pd(temp, sum2);
    result.x2 = temp[0];

    product1 = _mm_mul_pd(m1_x2, m2_col2);
    product2 = _mm_mul_pd(m1_x2_x, m2_col2_x);
    sum1 = _mm_add_pd(product1, product2);
    sum2 = _mm_hadd_pd(sum1, sum1);
    _mm_storeu_pd(temp, sum2);
    result.y2 = temp[0];

    product1 = _mm_mul_pd(m1_x2, m2_col3);
    product2 = _mm_mul_pd(m1_x2_x, m2_col3_x);
    sum1 = _mm_add_pd(product1, product2);
    sum2 = _mm_hadd_pd(sum1, sum1);
    _mm_storeu_pd(temp, sum2);
    result.z2 = temp[0];

    // Third row
    __m128d m1_x3 = _mm_set_pd(m1.z3, m1.y3);
    __m128d m1_x3_x = _mm_set_pd(0.0, m1.x3);
    
    product1 = _mm_mul_pd(m1_x3, m2_col1);
    product2 = _mm_mul_pd(m1_x3_x, m2_col1_x);
    sum1 = _mm_add_pd(product1, product2);
    sum2 = _mm_hadd_pd(sum1, sum1);
    _mm_storeu_pd(temp, sum2);
    result.x3 = temp[0];

    product1 = _mm_mul_pd(m1_x3, m2_col2);
    product2 = _mm_mul_pd(m1_x3_x, m2_col2_x);
    sum1 = _mm_add_pd(product1, product2);
    sum2 = _mm_hadd_pd(sum1, sum1);
    _mm_storeu_pd(temp, sum2);
    result.y3 = temp[0];

    product1 = _mm_mul_pd(m1_x3, m2_col3);
    product2 = _mm_mul_pd(m1_x3_x, m2_col3_x);
    sum1 = _mm_add_pd(product1, product2);
    sum2 = _mm_hadd_pd(sum1, sum1);
    _mm_storeu_pd(temp, sum2);
    result.z3 = temp[0];

    return result;
}



mat3x3 m33_product_m33(mat3x3 m1, mat3x3 m2){
    mat3x3 m;
    
    m.x1 = m1.x1 * m2.x1 + m1.y1*m2.x2 + m1.z1*m2.x3;
    m.x2 = m1.x2 * m2.x1 + m1.y2*m2.x2 + m1.z2*m2.x3;
    m.x3 = m1.x3 * m2.x1 + m1.y3*m2.x2 + m1.z3*m2.x3;

    m.y1 = m1.x1 * m2.y1 + m1.y1*m2.y2 + m1.z1*m2.y3;
    m.y2 = m1.x2 * m2.y1 + m1.y2*m2.y2 + m1.z2*m2.y3;
    m.y3 = m1.x3 * m2.y1 + m1.y3*m2.y2 + m1.z3*m2.y3;

    m.z1 = m1.x1 * m2.z1 + m1.y1*m2.z2 + m1.z1*m2.z3;
    m.z2 = m1.x2 * m2.z1 + m1.y2*m2.z2 + m1.z2*m2.z3;
    m.z3 = m1.x3 * m2.z1 + m1.y3*m2.z2 + m1.z3*m2.z3;

    return m;
}

vec3 m33_product_v3(mat3x3 m1, vec3 v2){
    vec3 vr;
    vr.x = m1.x1 * v2.x + m1.y1 * v2.y + m1.z1 * v2.z;
    vr.y = m1.x2 * v2.x + m1.y2 * v2.y + m1.z2 * v2.z;
    vr.z = m1.x3 * v2.x + m1.y3 * v2.y + m1.z3 * v2.z;
    return vr;

}


mat3x3 m33_transpose(mat3x3 m1){
    mat3x3 m = {
        m1.x1, m1.y1, m1.z1,
        m1.x2, m1.y2, m1.z2,
        m1.x3, m1.y3, m1.z3
    };

    return m;  
}



// Geometric transformations
mat3x3 m33_rotationX(double angle) {
    double c = cos(angle);
    double s = sin(angle);

    return (mat3x3){
        1, 0, 0,
        0, c, s,
        0, -s, c
    };
}


mat3x3 m33_rotationY(double angle){
    double s = sin(angle);
    double c = cos(angle);
    
    return (mat3x3) {
            c,  0, -s, 
            0,  1,  0,  
            s,  0,  c			 
    };
}

mat3x3 m33_rotationZ(double angle){
    double ca = cos(angle);
    double sa = sin(angle);
    
    mat3x3 res = {
        ca, -sa, 0,
        sa,  ca, 0,
        0,    0, 1
    };
    
    return res;

}

mat3x3 m33_rotation_XYZ(double alpha, double beta, double gamma){
    double ca = cos(alpha);
    double sa = sin(alpha);
    double cb = cos(beta);
    double sb = sin(beta);
    double cg = cos(gamma);
    double sg = sin(gamma);

    double ca_cb = ca*cb;
    double ca_sb = ca*sb;
    double sa_cb = sa*cb;
    double sa_sb = sa*sb;
    // Removed unused variables: sg_cb, sg_sb

    return (mat3x3){ 
        cb*cg, sa_sb*cg-ca_cb*sg, ca_sb*cg+sa_cb*sg, 
        sb,    sa_cb+sa_sb*sg, ca_cb-sa_sb*sg, 
        -sb,   sa*cg,          ca*cg                           
    };
}


mat3x3 m33_scale_xyz(double k){
    mat3x3 m ={
        k, 0.0f, 0.0f, 
        0.0f, k, 0.0f, 
        0.0f, 0.0f, k
    };
    return m;
    
}


mat3x3 m33_scale_xyz_v3(vec3 sxyz){
    mat3x3 sc = {sxyz.x, 0.0f, 0.0f,
                 0.0f, sxyz.y, 0.0f,
                 0.0f, 0.0f, sxyz.z};
    return sc;

}


vec3 v3_barycentric(vec3 P, vec3 A, vec3 B, vec3 C) {
     
    vec3 v0 = v3_diff(B,A); 
    vec3 v1 = v3_diff(C,A);
    vec3 v2 = v3_diff(P,A);

    double d00 = v0.x*v0.x + v0.y*v0.y + v0.z*v0.z;
    double d01 = v0.x*v1.x + v0.y*v1.y + v0.z*v1.z;
    
    double d11 = v1.x*v1.x + v1.y*v1.y + v1.z*v1.z;
    
    double d20 = v2.x*v0.x + v2.y*v0.y + v2.z*v0.z;
    double d21 = v2.x*v1.x + v2.y*v1.y + v2.z*v1.z;

    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;

    return (vec3){u,v,w};
}

bool is_point_within_cone(vec3 P, Cone C) {
    // A cone is characterised by the coordinates of its vertex, the direction vector of the axis and the aperture angle.
    vec3 diff = v3_diff(P, C.vertex);
    double diff_mod = v3_mod(diff);
    double axis_mod = v3_mod(C.axis);
    
    // Safety checks for zero-length vectors
    if(diff_mod < 1e-12 || axis_mod < 1e-12) {
        return false;
    }
    
    // vector pointing from the vertex to the point
    double dp = v3_dot(diff, C.axis);
    // compare to the cosine of the angle of opening
    double cos_theta = cos(C.aperture/2);
    return dp >= (dp * dp) / (diff_mod * axis_mod) * cos_theta;
}

double v3_angle_to_x(vec3 v){
    double vm = v3_mod(v);
    if(vm < 1e-12) return 0.0; // Near-zero check
    double ratio = fabs(v.x)/vm;
    // Clamp to avoid NaN from acos
    ratio = fmin(fmax(ratio, -1.0), 1.0);
    return acos(ratio);
}

double v3_angle_to_y(vec3 v){
    double vm = v3_mod(v);
    if(vm < 1e-12) return 0.0; // Near-zero check
    double ratio = fabs(v.y)/vm;
    // Clamp to avoid NaN from acos
    ratio = fmin(fmax(ratio, -1.0), 1.0);
    return acos(ratio);

}

double v3_angle_to_z(vec3 v){
    double vm = v3_mod(v);
    if(vm < 1e-12) return 0.0; // Near-zero check
    double ratio = fabs(v.z)/vm;
    // Clamp to avoid NaN from acos
    ratio = fmin(fmax(ratio, -1.0), 1.0);
    return acos(ratio);
}


vec3 v3_lerp(vec3 p1, vec3 p2, double t) {
    return (vec3){
        p1.x + (p2.x - p1.x) * t,
        p1.y + (p2.y - p1.y) * t,
        p1.z + (p2.z - p1.z) * t
    };
}
