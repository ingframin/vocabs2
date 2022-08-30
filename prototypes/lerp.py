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
    return v1+(v2-v1)*t
    
v1 = vec2(1,2)
v2 = vec2(8,5)

for i in range(11):
    print(lerp(v1,v2,i/10))
