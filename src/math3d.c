
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

__m128d v3_dot_simd(__m128d v1x, __m128d v1y, __m128d v1z, __m128d v2x, __m128d v2y, __m128d v2z){
    __m128d v1v2x = _mm_mul_pd(v1x, v2x);
    __m128d v1v2y = _mm_mul_pd(v1y, v2y);
    __m128d v1v2z = _mm_mul_pd(v1z, v2z);

    __m128d sum = _mm_add_pd(_mm_add_pd(v1v2x, v1v2y), v1v2z);

    return sum;
}

double v3_mod(vec3 v1){
    __m128d v1x = _mm_set1_pd(v1.x);
    __m128d v1y = _mm_set1_pd(v1.y);
    __m128d v1z = _mm_set1_pd(v1.z);

    __m128d v1v1 = v3_dot_simd(v1x, v1y, v1z, v1x, v1y, v1z);

    double* v1v1_arr = (double*)&v1v1;

    return sqrt(v1v1_arr[0] + v1v1_arr[1]);
}

double v3_distance(vec3 v1, vec3 v2){
    __m128d v1x = _mm_set1_pd(v1.x);
    __m128d v1y = _mm_set1_pd(v1.y);
    __m128d v1z = _mm_set1_pd(v1.z);
    __m128d v2x = _mm_set1_pd(v2.x);
    __m128d v2y = _mm_set1_pd(v2.y);
    __m128d v2z = _mm_set1_pd(v2.z);

    __m128d dx = _mm_sub_pd(v1x, v2x);
    __m128d dy = _mm_sub_pd(v1y, v2y);
    __m128d dz = _mm_sub_pd(v1z, v2z);

    __m128d sum = _mm_add_pd(_mm_add_pd(_mm_mul_pd(dx, dx), _mm_mul_pd(dy, dy)), _mm_mul_pd(dz, dz));

    double* sum_arr = (double*)&sum;

    return sqrt(sum_arr[0] + sum_arr[1] + sum_arr[2]);
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


// Refactored SIMD matrix multiplication function for mat3x3 structures
mat3x3 simd_m33_product_m33(mat3x3 m1, mat3x3 m2) {
    mat3x3 m;

    // Load the matrices into SIMD registers
    __m256d m1_row1 = _mm256_loadu_pd(&m1.x1);
    __m256d m1_row2 = _mm256_loadu_pd(&m1.x2);
    __m256d m1_row3 = _mm256_set_pd(m1.z1, m1.y2, m1.x3, 0.0);

    __m256d m2_row1 = _mm256_set_pd(0.0, m2.z1, m2.y1, m2.x1);
    __m256d m2_row2 = _mm256_set_pd(0.0, m2.z2, m2.y2, m2.x2);
    __m256d m2_row3 = _mm256_set_pd(0.0, m2.z3, m2.y3, m2.x3);

    // Perform the matrix multiplication using SIMD intrinsics
    __m256d result_row1 = _mm256_add_pd(
        _mm256_add_pd(_mm256_mul_pd(m1_row1, m2_row1), 
                      _mm256_mul_pd(m1_row2, m2_row2)), 
                      _mm256_mul_pd(m1_row3, m2_row3));

    __m128d result_low = _mm256_castpd256_pd128(result_row1);
    __m128d result_high = _mm256_extractf128_pd(result_row1, 1);

    // Store the result back into the struct
    _mm_storeu_pd(&m.x1, result_low);
    _mm_storeu_pd(&m.x2, result_high);

    return m;
}



mat3x3 m33_product_m33(mat3x3 m1, mat3x3 m2){
    __m256d m1_row1 = _mm256_loadu_pd(&m1.x1);
    __m256d m1_row2 = _mm256_loadu_pd(&m1.x2);
    __m256d m1_row3 = _mm256_set_pd(m1.z1, m1.y2, m1.x3, 0.0);

    __m256d m2_row1 = _mm256_loadu_pd(&m2.x1);
    __m256d m2_row2 = _mm256_loadu_pd(&m2.x2);
    __m256d m2_row3 = _mm256_set_pd(m2.z1, m2.y2, m2.x3, 0.0);

    __m256d result_row1 = _mm256_mul_pd(m1_row1, m2_row1);
    __m256d result_row2 = _mm256_mul_pd(m1_row2, m2_row2);
    __m256d result_row3 = _mm256_mul_pd(m1_row3, m2_row3);

    __m256d result_row1_sum = _mm256_hadd_pd(result_row1, result_row2);
    __m256d result_row2_sum = _mm256_hadd_pd(result_row3, _mm256_setzero_pd());

    __m128d result_low = _mm256_castpd256_pd128(result_row1_sum);
    __m128d result_high = _mm256_extractf128_pd(result_row1_sum, 1);
    __m128d result_high_2 = _mm256_castpd256_pd128(result_row2_sum);

    mat3x3 m;
    _mm_storeu_pd(&m.x1, _mm_add_pd(result_low, result_high_2));
    _mm_storeu_pd(&m.x2, result_high);

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
        {m1.x1, m1.y1, m1.z1},
        {m1.x2, m1.y2, m1.z2},
        {m1.x3, m1.y3, m1.z3}
    };

    return m;  
}



// Geometric transformations
mat3x3 m33_rotationX(double angle) {
    double c = cos(angle);
    double s = sin(angle);

    return (mat3x3){
        {1, 0, 0},
        {0, c, s},
        {0, -s, c}
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

//Precalculate some values
static const double halfPI = 0.5*M_PI;
static const double twoPI = 2.0*M_PI;

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
    double sg_cb = sg*cb;
    double sg_sb = sg*sb;

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
    mat3x3 sc = {{sxyz.x, 0.0f, 0.0f},
                 {0.0f, sxyz.y, 0.0f},
                 {0.0f, 0.0f, sxyz.z}};
    return sc;

}


vec3 v3_barycentric(vec3 P, vec3 A, vec3 B, vec3 C) {
    vec3 v0 = B - A; 
    vec3 v1 = C - A; 
    vec3 v2 = P - A;

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
    // vector pointing from the vertex to the point
    double dp = v3_dot(P - C.vertex, C.axis);
    // compare to the cosine of the angle of opening
    double cos_theta = cos(C.aperture/2);
    return dp >= dp * dp / (v3_mod(P - C.vertex) * v3_mod(C.axis)) * cos_theta;
}

double v3_angle_to_x(vec3 v){
    double vm = v3_mod(v);
    if(vm==0.0) return 0.0;
    return acos(fabs(v.x)/vm);
}

double v3_angle_to_y(vec3 v){
    double vm = v3_mod(v);
    if(vm==0.0) return 0.0;
    return acos(fabs(v.y)/vm);

}

double v3_angle_to_z(vec3 v){
    double vm = v3_mod(v);
    if(vm==0.0) return 0.0;
    return acos(fabs(v.z)/vm);
}


vec3 v3_lerp(vec3 p1, vec3 p2, double t) {
    return (vec3){
        p1.x + (p2.x - p1.x) * t,
        p1.y + (p2.y - p1.y) * t,
        p1.z + (p2.z - p1.z) * t
    };
}
