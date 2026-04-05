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

#include "math2d.h"
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510
#endif
#include <cstdlib>

// ======================
// vec2 Class Methods
// ======================

double vec2::mod() const
{
  return hypot(x, y);
}

vec2 vec2::normalize() const
{
  if(isZero(1e-12)) {
    // Return zero vector for zero-length input
    return vec2(0.0, 0.0);
  }
  
  double m = mod();
  return vec2(x / m, y / m);
}

vec2 vec2::operator+(const vec2& other) const
{
  return vec2(x + other.x, y + other.y);
}

vec2 vec2::operator-(const vec2& other) const
{
  return vec2(x - other.x, y - other.y);
}

vec2 vec2::operator*(double k) const
{
  return vec2(x * k, y * k);
}

vec2 vec2::operator+(double k) const
{
  return vec2(x + k, y + k);
}

bool vec2::operator==(const vec2& other) const
{
  return x == other.x && y == other.y;
}

bool vec2::operator!=(const vec2& other) const
{
  return !(*this == other);
}

bool vec2::isZero(double epsilon) const
{
  return fabs(x) < epsilon && fabs(y) < epsilon;
}

// ======================
// Vector Operations (free functions)
// ======================

double v2_dot(const vec2& v1, const vec2& v2)
{
  return v1.x * v2.x + v1.y * v2.y;
}

// ======================
// Geometric Transformations
// ======================

vec2 v2_rotate(const vec2& v, double angle)
{
  double cos_theta = cos(angle);
  double sin_theta = sin(angle);
  
  return vec2(
    v.x * cos_theta - v.y * sin_theta,
    v.x * sin_theta + v.y * cos_theta
  );
}

vec2 v2_rotateLeftHalfPI(const vec2& v)
{
  return vec2(-v.y, v.x);
}

vec2 v2_rotateRightHalfPI(const vec2& v)
{
  return vec2(v.y, -v.x);
}

vec2 v2_reverse(const vec2& v)
{
  return vec2(-v.x, -v.y);
}

// ======================
// Distance and Angle Calculations
// ======================

double v2_distance(const vec2& v1, const vec2& v2)
{
  double dx = v2.x - v1.x;
  double dy = v2.y - v1.y;
  return hypot(dx, dy);
}

double v2_angle_between(const vec2& v1, const vec2& v2)
{
  double dot = v2_dot(v1, v2);
  double mod1 = v1.mod();
  double mod2 = v2.mod();
  
  // Handle division by zero
  if (mod1 < 1e-12 || mod2 < 1e-12) {
    return 0.0;
  }
  
  double cos_theta = dot / (mod1 * mod2);
  
  // Clamp to avoid floating point errors
  if (cos_theta > 1.0) cos_theta = 1.0;
  if (cos_theta < -1.0) cos_theta = -1.0;
  
  return acos(cos_theta);
}

// ======================
// Interpolation
// ======================

vec2 v2_lerp(const vec2& p1, const vec2& p2, double t)
{
  // Clamp t to [0, 1] range
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  
  return vec2(
    p1.x + t * (p2.x - p1.x),
    p1.y + t * (p2.y - p1.y)
  );
}

vec2 v2_qspline(const vec2& p1, const vec2& p2, const vec2& p3, double t)
{
  // Clamp t to [0, 1] range
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  
  // Quadratic spline: P(t) = (1-t)^2 * P1 + 2*(1-t)*t * P2 + t^2 * P3
  double t2 = t * t;
  double one_minus_t = 1.0 - t;
  double one_minus_t2 = one_minus_t * one_minus_t;
  
  return vec2(
    one_minus_t2 * p1.x + 2 * one_minus_t * t * p2.x + t2 * p3.x,
    one_minus_t2 * p1.y + 2 * one_minus_t * t * p2.y + t2 * p3.y
  );
}

vec2 v2_cspline(const vec2& p1, const vec2& p2, const vec2& p3, const vec2& p4, double t)
{
  // Clamp t to [0, 1] range
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  
  // Cubic spline using Catmull-Rom formula
  double t2 = t * t;
  double t3 = t2 * t;
  
  double b0 = -0.5 * t3 + t2 - 0.5 * t;
  double b1 = 1.5 * t3 - 2.5 * t2 + 1.0;
  double b2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t;
  double b3 = 0.5 * t3 - 0.5 * t2;
  
  return vec2(
    b0 * p1.x + b1 * p2.x + b2 * p3.x + b3 * p4.x,
    b0 * p1.y + b1 * p2.y + b2 * p3.y + b3 * p4.y
  );
}

vec2* v2_interpolate(const vec2 vs[], size_t vs_len, double t)
{
  if (vs_len == 0) return NULL;
  
  // Clamp t to [0, 1] range
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  
  // Allocate array for interpolated points
  vec2* result = new vec2[vs_len];
  
  // Use de Casteljau's algorithm for each segment
  for (size_t i = 0; i < vs_len - 1; i++) {
    vec2 p0 = vs[i];
    vec2 p1 = vs[i + 1];
    
    // Linear interpolation for now (could be enhanced)
    result[i] = v2_lerp(p0, p1, t);
  }
  
  // Last point is the same as input
  if (vs_len > 0) {
    result[vs_len - 1] = vs[vs_len - 1];
  }
  
  return result;
}

void v2_free_interpolated(vec2* points)
{
  if (points != NULL) {
    delete[] points;
  }
}

// ======================
// Barycentric Coordinates
// ======================

barycoords v2_barycentric(const vec2& A, const vec2& B, const vec2& C, const vec2& P)
{
  // Compute vectors
  vec2 v0 = B - A;
  vec2 v1 = C - A;
  vec2 v2 = P - A;
  
  // Compute dot products
  double dot00 = v2_dot(v0, v0);
  double dot01 = v2_dot(v0, v1);
  double dot02 = v2_dot(v0, v2);
  double dot11 = v2_dot(v1, v1);
  double dot12 = v2_dot(v1, v2);
  
  // Compute barycentric coordinates
  double inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
  double beta = (dot11 * dot02 - dot01 * dot12) * inv_denom;
  double gamma = (dot00 * dot12 - dot01 * dot02) * inv_denom;
  double alpha = 1.0 - beta - gamma;
  
  return barycoords(alpha, beta, gamma);
}

// ======================
// Obstacle Calculations (Velocity Obstacle Method)
// ======================

Obstacle compute_obstacle(const vec2& pos1, const vec2& pos2, double size1, double size2)
{
  Obstacle obs;
  
  // Compute center position (midpoint between agents)
  obs.position = vec2(
    (pos1.x + pos2.x) / 2.0,
    (pos1.y + pos2.y) / 2.0
  );
  
  // Compute radius (sum of agent radii)
  obs.radius = size1 + size2;
  
  // Compute direction vector between agents
  vec2 dir = pos2 - pos1;
  double dist = dir.mod();
  
  // Handle case where agents are at the same position
  if (dist < 1e-12) {
    // Return a default obstacle
    obs.T1 = vec2(obs.position.x + obs.radius, obs.position.y);
    obs.T2 = vec2(obs.position.x - obs.radius, obs.position.y);
    return obs;
  }
  
  // Normalize direction vector
  dir = dir.normalize();
  
  // Compute tangent points (perpendicular to direction)
  vec2 perp = vec2(-dir.y, dir.x); // 90 degree rotation
  
  obs.T1 = obs.position + (perp * obs.radius);
  obs.T2 = obs.position + (perp * (-obs.radius));
  
  return obs;
}