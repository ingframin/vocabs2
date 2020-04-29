#include "vec2.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

inline double v2_mod(vec2* v){
  return sqrt(v->x * v->x + v->y * v->y);
}

inline vec2 v2_rotate(vec2* v,double angle){
  
   vec2 ret; 
   ret.x = v->x * cos(angle) - v->y * sin(angle);
   ret.y = v->x * sin(angle) + v->y * cos(angle);
   return ret;

}

inline vec2 v2_norm(vec2* v){
  vec2 n;
  double m = v2_mod(v);
  n.x = v->x/m;
  n.y = v->y/m;
  return n;
  
}

inline vec2 v2_add(vec2* v1,vec2* v2){
  vec2 res;
  res.x = v1->x+v2->x;
  res.y = v1->y+v2->y;
  return res;
}

inline vec2 v2_sub(vec2* v1, vec2* v2){
  vec2 res;
  res.x = v1->x-v2->x;
  res.y = v1->y-v2->y;
  return res;
}

inline double v2_dot(vec2* v1,vec2* v2){
  return v1->x*v2->x + v1->y*v2->y;
}

inline vec2 v2_addK(vec2* v,double k){
  vec2 res;
  res.x = v->x+k;
  res.y = v->y+k;
  return res;
}

inline vec2 v2_prodK(vec2* v, double k){
  vec2 res;
  res.x = v->x*k;
  res.y = v->y*k;
  return res;
}

inline double v2_distance(vec2* v1, vec2* v2){
  vec2 res =  v2_sub(v1,v2);
  return v2_mod(&res);
}
