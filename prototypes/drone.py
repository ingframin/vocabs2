from vector2 import *
from dataclasses import dataclass
from collections import deque
from math import sqrt,cos,sin,acos

@dataclass
class Obstacle:
    size :float
    pos :vec2
    T1 :vec2
    T2 :vec2

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

class Drone:
    def __init__(self, id, x, y, vx, vy, size) -> None:
        self.id = id
        self.pos = vec2(x,y)
        self.vel = vec2(vx,vy)
        self.size = size
        self.trajectory = deque()
    
    def move(self,dt):
        self.pos += self.vel*dt

    def accelerate(self,dv,dt):
        self.vel += dv*dt

    def stop(self):
        self.vel.x = 0
        self.vel.y = 0

    def start(self,vx,vy):
        self.vel = vec2(vx,vy)    
    
    def collide(self,drone):
        obs = self.obstacle(drone)
        # (V1 - V2) + P (The translation is needed to check if the difference falls into the triangle)
        DV = self.vel - drone.vel + self.pos
        # computes barycentric coordinates
        a,b,g = barycentric(obs.T1, obs.T2, self.pos, DV.x, DV.y)
        return (a>0 and b>0 and g>0)

    def compute_trajectory(self,P, steps=100):
        
        Pm1 = self.vel*self.size
        try:
            pstart = self.trajectory[-1]
        except:
            pstart = self.pos

        for i in range(1,steps+1): #11 steps just because...
            self.trajectory.append(spline(pstart,Pm1,P,i/steps))
        
    def obstacle(self, d2):
        # Minkowski addition
        r = self.size+d2.size

        # Computing tangent lines to circle passing through the point self.pos
        dx = self.pos.x-d2.pos.x

        a = dx**2 - r**2
        b = 2*dx*(d2.pos.y-self.pos.y)
        c = (d2.pos.y-self.pos.y)**2 - r**2
        Delta = b**2 - 4*a*c
        if Delta < 0:
            Delta = 0
        # Angular coefficient
        m1 = (-b + sqrt(Delta))/(2*a)
        m2 = (-b - sqrt(Delta))/(2*a)
       
        # Intersection with y axis
        q1 = self.pos.y-m1*self.pos.x
        q2 = self.pos.y-m2*self.pos.x

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
        return Obstacle(r, d2.pos,vec2(xt1,yt1),vec2(xt2,yt2))
        
    def steer_towards(self,P):
        M = self.vel.mod()
        dirp = (P-self.pos).norm()
        
        self.vel = dirp*M

