#ifndef VEC2_H
#define VEC2_H

struct vec2
{
  double x;
  double y;
  //module
  double mod()const;
  //rotate
  vec2 rotate(double angle) const;
  vec2 rotate(const vec2& v) const;
  //rotate sign * PI/2
  vec2 rotateHalfPI(int sign)const;
  //normalize
  vec2 norm() const;
  //Add and subtract
  vec2 add(const vec2& v2)const;
  vec2 sub(const vec2& v2)const;
  //dot product
  double dot(const vec2& v2)const;
  //add constant to both elements
  vec2 addK(double k)const;
  //multiply both elements by k
  vec2 prodK(double k)const;
  //distance between 2 vectors
  double distance(const vec2& v2)const;
};



#endif