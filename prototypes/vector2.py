from math import sqrt, cos, sin, pi, acos

class vec2:
    def __init__(self, x,y):
        self.x = x
        self.y = y

    def __add__(self,v2):
        return vec2(self.x+v2.x,self.y+v2.y)
    
    def __sub__(self,v2):
        return vec2(self.x-v2.x,self.y-v2.y)

    def __mul__(self,k):
        if type(k) == vec2:
            return self.x*k.x+self.y*k.y
        return vec2(self.x*k,self.y*k)
    
    def mod(self):
        return sqrt(self.x**2 + self.y**2)
    
    def distance(self, v2):
        
        return sqrt((self.x-v2.x)**2 + (self.y-v2.y)**2)
    
    def norm(self):
        module = self.mod()
        return vec2(self.x/module, self.y/module)
    
    def scale(self, k):
        return vec2(self.x*k,self.y*k)

    def angle_to(self,v2):
        v1n = self.norm()
        v2n = v2.norm()
        C = v1n*v2n/(v1n.mod()*v2n.mod())
        if abs(C)>1:
            C = C/abs(C)
        return acos(C)

    def rotate(self,angle):
        match abs(angle):
            case 
        if abs(angle) < 1e-4:
            return self
        elif abs(angle - pi/2) <1e-4:
            return vec2(-self.y,self.x)
        elif abs(angle - pi) <1e-4:
            return vec2(-self.x,-self.y)
        elif abs(angle - 3*pi/2) <1e-4:
            return vec2(self.y,-self.x)
            
        m = self.mod()
        X = self.x / m
        Y = self.y / m
        C = cos(angle)
        S = sin(angle)
        return vec2(m * (X * C - Y * S),m * (X * S + Y * C))

    def __str__(self):
        return f'{self.x},{self.y}'

    def __repr__(self):
        return f'{self.x},{self.y}'
    


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
