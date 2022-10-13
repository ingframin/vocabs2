
from dataclasses import dataclass
from collections import deque
from math import sqrt,pi,copysign
from geometry import Vec2f, magnitude,norm,distance,spline,scale,barycentric,Obstacle

class Drone:
    def __init__(self, id, x, y, vx, vy, size) -> None:
        self.id = id
        self.pos = Vec2f(x,y)
        self.vel = Vec2f(vx,vy)
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
        self.vel = Vec2f(vx,vy)    
    
    
    def will_collide(self,obstacle):
        
        # (V1 - V2) + P (The translation is needed to check if the difference falls into the triangle)
        DV = self.vel - obstacle.vel + self.pos
        # computes barycentric coordinates
        a,b,g = barycentric(obstacle.T1, obstacle.T2, self.pos, DV.x, DV.y)
        return (a>0 and b>0 and g>0)

    def compute_trajectory(self,P, steps=100):
        
        Pm1 = self.pos+P
        M = magnitude(Pm1)/2
        
        Pm1 = scale(norm(Pm1),M)
        try:
            pstart = self.trajectory[-1]
        except:
            pstart = self.pos

        for i in range(1,steps+1): 
            self.trajectory.append(spline(pstart,Pm1,P,i/steps))
    
    def insert_trajectory(self,P, steps=200):
        
        Pm1 = self.vel
        pstart = self.pos

        for i in range(1,steps+1): 
            self.trajectory.appendleft(spline(pstart,Pm1,P,i/steps))
        
    def steer_towards(self,P):
        
        M = magnitude(self.vel)
        
        dirp = norm(P-self.pos)

        self.vel = dirp*M

    def has_next_point(self):
        return len(self.trajectory)>0
    
    def reached(self,point):
        return distance(self.pos,point) < self.size

    def current_target(self):
        return self.trajectory[0]

    def pop_waypoint(self):
        return self.trajectory.popleft()

    def compute_avoidance(self,drones):
        obstacles = [Obstacle(self,d) for d in drones if d.id != self.id]
        v = self.vel
        while any([self.will_collide(obs) for obs in obstacles]):
            self.vel = self.vel.rotate(2*pi/(10*len(obstacles)))
        p = self.pos+self.vel*2
        self.vel = v
        self.insert_trajectory(p)
