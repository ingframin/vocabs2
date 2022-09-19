module Geometry where

data Vec2D = Vec2D Double Double deriving (Show) 

v2_add:: Vec2D -> Vec2D -> Vec2D
v2_add (Vec2D x1 y1) (Vec2D x2 y2) = Vec2D (x1+x2) (y1+y2)

v2_sub:: Vec2D -> Vec2D -> Vec2D
v2_sub (Vec2D x1 y1) (Vec2D x2 y2) = Vec2D (x1-x2) (y1-y2)

v2_mag:: Vec2D -> Double
v2_mag (Vec2D x y) = sqrt (x**2 + y**2)

v2_rotate:: Vec2D -> Double -> Vec2D
v2_rotate (Vec2D x y) angle =   let m = v2_mag (Vec2D x y)
                                    c = cos(angle)
                                    s = sin(angle)
                                in Vec2D (m * (x * c - y * s)) (m * (x * s + y * c))

v2_norm:: Vec2D -> Vec2D
v2_norm (Vec2D x y) = let m = v2_mag (Vec2D x y)
                      in Vec2D (x/m) (y/m)


v2_scale:: Vec2D -> Double -> Vec2D
v2_scale (Vec2D x y) k = Vec2D (x*k) (y*k)

v2_dot:: Vec2D -> Vec2D -> Double
v2_dot (Vec2D x1 y1) (Vec2D x2 y2) = x1*x2+y1*y2

v2_lerp:: Vec2D -> Vec2D -> Double -> Vec2D
v2_lerp v1 v2 t =   let v1 = v2_scale v1 (1.0-t)
                        v2 = v2_scale v2 t
                    in v2_add v1 v2 

v2_spline::Vec2D -> Vec2D -> Vec2D -> Double -> Vec2D
v2_spline v1 v2 v3 t =  let p0 = v2_lerp v1 v2 t
                            p1 = v2_lerp v2 v3 t
                        in v2_lerp p0 p1 t
