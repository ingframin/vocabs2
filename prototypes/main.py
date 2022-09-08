from math import sqrt
import pygame as pg
from drone import *

def draw_drone(d1, screen):

    dx = d1.pos.x
    dy = d1.pos.y
    r = d1.size

    screen.blit(drone_gfx1, (dx-r/2, dy-r/2))

    pg.draw.circle(screen, (255, 255, 0, 255), (round(dx), round(dy)), r*1.2,2)
    
    dvx = dx+d1.vel.x*5
    dvy = dy+d1.vel.y*5

    pg.draw.line(screen, (0, 0, 255, 255), (round(dx), round(dy)), (round(dvx), round(dvy)), 2)
    
pg.init()
dt = 1E-3
dsize = 30
clock = pg.time.Clock()
size = width, height = 800, 800
black = 0, 0, 0

screen = pg.display.set_mode(size)
drone_gfx1 = pg.transform.scale(pg.image.load('drone.png'), (dsize, dsize))

running = True
d1 = Drone(0,200,200,30,30,dsize)
print(d1.pos)
P = None
while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
            break
        elif event.type == pg.MOUSEBUTTONDOWN:
            x,y = pg.mouse.get_pos()
            d1.compute_trajectory(vec2(x,y),steps=400)
            
    if len(d1.trajectory) > 0:
        P = d1.trajectory.popleft()
        d1.steer_towards(P)
    
    d1.move(1/60)
    screen.fill(black)
    draw_drone(d1,screen)
    pg.display.flip()
    clock.tick(60)

pg.quit()