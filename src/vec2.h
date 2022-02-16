#ifndef VEC2_H
#define VEC2_H

struct vec2
{
  double x;
  double y;
  //module
  double mod();
  //rotate
  vec2 rotate(double angle);
  vec2 rotate(vec2 v);
  //rotate sign * PI/2
  vec2 rotateHalfPI(int sign);
  //normalize
  vec2 norm();
  //Add and subtract
  vec2 add(vec2 v2);
  vec2 sub(vec2 v2);
  //dot product
  double dot(vec2 v2);
  //add constant to both elements
  vec2 addK(double k);
  //multiply both elements by k
  vec2 prodK(double k);
  //distance between 2 vectors
  double distance(vec2 v2);
};



#endif
