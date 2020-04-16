#ifndef VEC2_H
#define VEC2_H
#define DROUND(x) (int)(x+0.5)

typedef struct {
  double x;
  double y;

} vec2;

double v2_mod(vec2*);
vec2 v2_rotate(vec2*,double angle);
vec2 v2_norm(vec2*);
vec2 v2_add(vec2*,vec2*);
vec2 v2_sub(vec2*,vec2*);
double v2_dot(vec2*,vec2*);
vec2 v2_addK(vec2*,double k);
vec2 v2_prodK(vec2*, double k);
double v2_distance(vec2*,vec2*);
#endif
