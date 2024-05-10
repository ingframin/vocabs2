
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



Cone newCone(vec3 Sc, double radius, vec3 P){
    Cone c;
    c.vertex = P;
    vec3 dif = v3_diff(P,Sc);
    double d = v3_mod(dif);
    double l = sqrt(d*d - radius*radius);
    c.aperture = 2*asin(radius/d);
    double h = l*cos(c.aperture/2);
    vec3 cp = v3_diff(Sc,P);
    c.axis = v3_scale(cp,h);
    return c;
}



vec3 v3_add(vec3 v1, vec3 v2){
    vec3 v = {v1.x+v2.x,v1.y+v2.y,v1.z+v2.z};
    return v;
}

vec3 v3_diff(vec3 v1, vec3 v2){
    vec3 v = {v1.x-v2.x,v1.y-v2.y,v1.z-v2.z};
    return v;
}

double v3_dot(vec3 v1, vec3 v2){
    return v1.x*v2.x+v1.y*v2.y+v1.z*v2.z;
}

double v3_mod(vec3 v1){
    return sqrt(v3_dot(v1,v1));
}

double v3_distance(vec3 v1, vec3 v2){
    vec3 dif = v3_diff(v1,v2);
    return v3_mod(dif);
}

double v3_angle_between(vec3 v1, vec3 v2){
    double dp = v3_dot(v1,v2);
    double mag1 = v3_mod(v1);
    double mag2 = v3_mod(v2);
    double C = dp/(mag1*mag2);
    if(C >= 1.0){
        return 0.0;
    }
    else if(C<=-1.0){
        return M_PI;
    }

    return acos(C);

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

    vec3 angles = {acos(cx),acos(cy),acos(cz)};
    return angles;
}

vec3 v3_cross(vec3 v1, vec3 v2){
    vec3 v = {v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x};
    return v;
}

vec3 v3_normalize(vec3 v1){
    double mag = v3_mod(v1);
    vec3 v = {v1.x/mag, v1.y/mag, v1.z/mag};
    return v;
}

vec3 v3_scale(vec3 v, double k){
    vec3 sv = {v.x*k, v.y*k, v.z*k}; 
    return sv;
}


mat3x3 simd_m33_product_m33(mat3x3 m1, mat3x3 m2) {
    mat3x3 m;

    // Load the matrices into SIMD registers
    __m256d m1_row1 = _mm256_setr_pd(m1.x1, m1.y1, 0.0, 0.0);
    __m256d m1_row2 = _mm256_setr_pd(m1.z1, m1.x2, 0.0, 0.0);
    __m256d m1_row3 = _mm256_setr_pd(m1.y2, m1.z2, 0.0, 0.0);

    __m256d m2_row1 = _mm256_setr_pd(m2.x1, m2.y1, 0.0, 0.0);
    __m256d m2_row2 = _mm256_setr_pd(m2.z1, m2.x2, 0.0, 0.0);
    __m256d m2_row3 = _mm256_setr_pd(m2.y2, m2.z2, 0.0, 0.0);

    // Perform the matrix multiplication using SIMD intrinsics
    __m256d result_row1 = _mm256_add_pd(
        _mm256_add_pd(_mm256_mul_pd(m1_row1, _mm256_permute4x64_pd(m2_row1, _MM_SHUFFLE(0, 0, 0, 0))), 
                      _mm256_mul_pd(m1_row2, _mm256_permute4x64_pd(m2_row1, _MM_SHUFFLE(1, 1, 1, 1)))), 
                      _mm256_mul_pd(m1_row3, _mm256_permute4x64_pd(m2_row1, _MM_SHUFFLE(2, 2, 2, 2))));

    __m256d result_row2 = _mm256_add_pd(
        _mm256_add_pd(_mm256_mul_pd(m1_row1, _mm256_permute4x64_pd(m2_row2, _MM_SHUFFLE(0, 0, 0, 0))), 
                      _mm256_mul_pd(m1_row2, _mm256_permute4x64_pd(m2_row2, _MM_SHUFFLE(1, 1, 1, 1)))), 
                      _mm256_mul_pd(m1_row3, _mm256_permute4x64_pd(m2_row2, _MM_SHUFFLE(2, 2, 2, 2))));

    __m256d result_row3 = _mm256_add_pd(
        _mm256_add_pd(_mm256_mul_pd(m1_row1, _mm256_permute4x64_pd(m2_row3, _MM_SHUFFLE(0, 0, 0, 0))), 
                      _mm256_mul_pd(m1_row2, _mm256_permute4x64_pd(m2_row3, _MM_SHUFFLE(1, 1, 1, 1)))), 
                      _mm256_mul_pd(m1_row3, _mm256_permute4x64_pd(m2_row3, _MM_SHUFFLE(2, 2, 2, 2))));

    // Store the result back into the struct
    _mm256_storeu_pd(&m.x1, result_row1);
    _mm256_storeu_pd(&m.x2, result_row2);
    _mm256_storeu_pd(&m.x3, result_row3);

    return m;
}



mat3x3 m33_product_m33(mat3x3 m1, mat3x3 m2){
    mat3x3 m;
    //row 1 
    m.x1 = m1.x1 * m2.x1 + m1.y1 * m2.x2 + m1.z1 * m2.x3;
    m.y1 = m1.x1 * m2.y1 + m1.y1 * m2.y2 + m1.z1 * m2.y3;
    m.z1 = m1.x1 * m2.z1 + m1.y1 * m2.z2 + m1.z1 * m2.z3;
    //row 2 
    m.x2 = m1.x2 * m2.x1 + m1.y2 * m2.x2 + m1.z2 * m2.x3;
    m.y2 = m1.x2 * m2.y1 + m1.y2 * m2.y2 + m1.z2 * m2.y3;
    m.z2 = m1.x2 * m2.z1 + m1.y2 * m2.z2 + m1.z2 * m2.z3;
    //row 3
    m.x3 = m1.x3 * m2.x1 + m1.y3 * m2.x2 + m1.z3 * m2.x3;
    m.y3 = m1.x3 * m2.y1 + m1.y3 * m2.y2 + m1.z3 * m2.y3;
    m.z3 = m1.x3 * m2.z1 + m1.y3 * m2.z2 + m1.z3 * m2.z3;

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
    mat3x3 m;

    m.x1 = m1.x1;
    m.y1 = m1.x2;
    m.z1 = m1.x3; 

    m.x2 = m1.y1;
    m.y2 = m1.y2;
    m.z2 = m1.y3;

    m.x3 = m1.z1;
    m.y3 = m1.z2;
    m.z3 = m1.z3;

    return m;  
}



// Geometric transformations
mat3x3 m33_rotationX(double angle){
    double ca = cos(angle);
    double sa = sin(angle);
    
            
    mat3x3 res = {
        1, 0,  0,  
        0, ca, -sa,
        0, sa, ca			
    };
    
    return res;
}

mat3x3 m33_rotationY(double angle){
    double ca = cos(angle);
    double sa = sin(angle);
    
    mat3x3 res = {
            ca, 0, sa, 
            0,  1, 0,  
        -sa, 0, ca			 
    };
    
    return res;
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

    mat3x3 res = {
        cb*cg, sa*sb*cg-ca*sg, ca*sb*cg+sa*sg, 
        cb*sg, sa*sb*sg+ca*cg, ca*sb*sg-sa*cg, 
        -sb,   sa*cb,          ca*cb                           
    };
    return res;
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
    double sx = sxyz.x, sy=sxyz.y, sz=sxyz.z;
    mat3x3 sc = {
        sx, 0.0f, 0.0f, 
        0.0f, sy, 0.0f, 
        0.0f, 0.0f, sz
    };
    return sc;

}


vec3 v3_barycentric(vec3 P, vec3 A, vec3 B, vec3 C) {
    vec3 v0 = v3_diff(B,A); 
    vec3 v1 = v3_diff(C,A); 
    vec3 v2 = v3_diff(P,A);

    double d00 = v3_dot(v0,v0);
    double d01 = v3_dot(v0,v1);
    
    double d11 = v3_dot(v1,v1);
    
    double d20 = v3_dot(v2,v0);
    double d21 = v3_dot(v2,v1);

    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;

    vec3 bar = {u,v,w};
    return bar;
}

bool is_point_within_cone(vec3 P, Cone C) {
    // A cone is characterised by the coordinates of its vertex, the direction vector of the axis and the aperture angle.
    // vector pointing from the vertex to the point
    vec3 VtoP = v3_diff(C.vertex,P);
    // v3_dot product of this vector with the cone axis
    double dp = v3_dot(VtoP,C.axis);
    // cosine of the angle between the two vectors
    double cos_angle = dp / sqrt(v3_mod(VtoP) * v3_mod(C.axis));
    // compare to the cosine of the angle of opening
    double cos_theta = cos(C.aperture/2);
    return cos_angle >= cos_theta;
}

double v3_angle_to_x(vec3 v){
    return acos(fabs(v.x)/v3_mod(v));
}

double v3_angle_to_y(vec3 v){
    return acos(fabs(v.y)/v3_mod(v));
}

double v3_angle_to_z(vec3 v){
    return acos(fabs(v.z)/v3_mod(v));
}


vec3 v3_lerp(vec3 p1, vec3 p2, double t){
    vec3 a = v3_scale(p1,1.0-t);
    vec3 b = v3_scale(p2,t);
    return v3_add(a,b);
}