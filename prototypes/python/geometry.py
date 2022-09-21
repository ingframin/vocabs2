from dataclasses import dataclass
from math import sqrt, cos, sin, pi, acos

@dataclass(frozen=True)
class Vec2f:
    x:float
    y:float

    def __add__(self,v2):
        return Vec2f(self.x+v2.x,self.y+v2.y)
    
    def __sub__(self,v2):
        return Vec2f(self.x-v2.x,self.y-v2.y)
    
    def __str__(self):
        return f'{self.x},{self.y}'

    def __repr__(self):
        return f'{self.x},{self.y}'

    def __len__(self):
        return sqrt(self.x**2 + self.y**2)

def dot(v1,v2):
    return Vec2f(v1.x*v2.x,v1.y*v2.y)

def mod(v1):
    return sqrt(v1.x**2 + v1.y**2)

def distance(v1, v2):
    
    return sqrt((v1.x-v2.x)**2 + (v1.y-v2.y)**2)

def norm(v1):
    module = len(v1)
    return Vec2f(v1.x/module, v1.y/module)

def scale(v1, k):
    return Vec2f(v1.x*k,v1.y*k)    

def rotate(v1,angle):
    m = len(v1)
    X = v1.x / m
    Y = v1.y / m
    C = cos(angle)
    S = sin(angle)
    return Vec2f(m * (X * C - Y * S),m * (X * S + Y * C))

def lerp(v1, v2, t):
    return v1*(1.0-t)+v2*t

def spline(v1,v2,v3,t):
    p0 = lerp(v1,v2,t)
    p1 = lerp(v2,v3,t)
    p = lerp(p0,p1,t)
    return p

def qspline(v1,v2,v3,v4,t):
    p0 = lerp(v1,v2,t)
    p1 = lerp(v2,v3,t)
    p2 = lerp(v3,v4,t)
    p = spline(p0,p1,p2,t)
    return p

@dataclass
class Obstacle:
    size :float
    pos :Vec2f
    vel: Vec2f
    T1 :Vec2f
    T2 :Vec2f
    
    def __init__(self,d1, d2):
        # Minkowski addition
        r = d1.size+d2.size

        # Computing tangent lines to circle passing through the point d1.pos
        dx = d1.pos.x-d2.pos.x

        a = dx**2 - r**2
        b = 2*dx*(d2.pos.y-d1.pos.y)
        c = (d2.pos.y-d1.pos.y)**2 - r**2
        Delta = b**2 - 4*a*c
        if Delta < 0:
            Delta = 0
        # Angular coefficient
        m1 = (-b + sqrt(Delta))/(2*a)
        m2 = (-b - sqrt(Delta))/(2*a)
       
        # Intersection with y axis
        q1 = d1.pos.y-m1*d1.pos.x
        q2 = d1.pos.y-m2*d1.pos.x

        # P1 - first tangent point.
        a1 = 1+m1**2
        b1 = 2*m1*q1-2*d2.pos.x-m1*2*d2.pos.y

        xt1 = (-b1)/(2*a1)
        yt1 = m1*xt1+q1

        # P2 - second tangent point.
        a2 = 1+m2**2
        b2 = 2*m2*q2-2*d2.pos.x-m2*2*d2.pos.y

        xt2 = (-b2)/(2*a2)
        yt2 = m2*xt2+q2
        
        self.size = r
        self.pos = d2.pos
        self.vel = d2.vel
        self.T1 = Vec2f(xt1,yt1)
        self.T2 = Vec2f(xt2,yt2)
        


#kinematic model of the drone
def barycentric( P1,  P2,  P3,  P):
    

    dg = ((P1.y - P2.y) * P3.x + (P2.x - P1.x) * P3.y + P1.x * P2.y - P2.x * P1.y)
    db = ((P1.y - P3.y) * P2.x + (P3.x - P1.x) * P2.y + P1.x * P3.y - P3.x * P1.y)

    if db == 0 or dg == 0:
        return (-1,-1,-1)
    gamma = ((P1.y - P2.y) * P.x + (P2.x - P1.x) * P.y + P1.x * P2.y - P2.x * P1.y) / dg
        
    beta = ((P1.y - P3.y) * P.x + (P3.x - P1.x) * P.y + P1.x * P3.y - P3.x * P1.y) / db
        
    alpha = 1 - beta - gamma
    return alpha, beta, gamma