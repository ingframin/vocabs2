from matplotlib import pyplot as plt
from random import randint

class vec2:
    def __init__(self, x,y):
        self.x = x
        self.y = y

    def __add__(self,v2):
        return vec2(self.x+v2.x,self.y+v2.y)
    
    def __sub__(self,v2):
        return vec2(self.x-v2.x,self.y-v2.y)

    def __mul__(self,k):
        return vec2(self.x*k,self.y*k)
    
    def __str__(self):
        return f'{self.x};{self.y}'

    def __repr__(self):
        return f'{self.x};{self.y}'

def lerp(v1, v2, t):
    return v1*(1.0-t)+v2*t

def spline(v1,v2,v3,t):
    p0 = lerp(v1,v2,t)
    p1 = lerp(v2,v3,t)
    p = lerp(p0,p1,t)
    return p

v1 = vec2(1,2)
v2 = vec2(-4,4)
v3 = vec2(8,5)
lx = []
ly = []
sx = []
sy = []


for i in range(11):
    lp = lerp(v1,v3,i/10)
    sp = spline(v1,v2,v3,i/10)
    lx.append(lp.x)
    ly.append(lp.y)
    sx.append(sp.x)
    sy.append(sp.y)

    
plt.plot([v1.x,v2.x,v3.x],[v1.y,v2.y,v3.y],'or')
plt.plot(lx,ly,'-*')
plt.plot(sx,sy,'-D')

plt.show()
