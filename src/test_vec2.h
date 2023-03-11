#ifndef TEST_V2
#define TEST_V2
#include "vec2.h"
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
//module
bool test_v2_mod();

//rotate sign * PI/2
bool test_v2_rotateHalfPI();

//rotate PI
bool test_v2_rotatePI();

//rotate
bool test_v2_rotate();

//normalize
bool test_v2_normalize();

//Add and subtract
bool test_v2_add();
bool test_v2_diff();

//dot product
bool test_v2_dot();

//add constant to both elements
bool test_v2_addK();

//multiply both elements by k
bool test_v2_scale();

//distance between 2 vectors
bool test_v2_distance();

//linear interpolation between 2 points
bool test_v2_lerp();

//Quadratic spline
bool test_v2_qspline();

//Cubic spline
bool test_v2_cspline();

//N-points interpolation
bool test_interpolate();

//Angle between 2 vectors
bool test_v2_angle_between();

#endif