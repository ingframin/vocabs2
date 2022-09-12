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
drones = [Drone(0,200,200,30,30,dsize),Drone(0,200,200,30,30,dsize),Drone(0,200,200,30,30,dsize),Drone(0,200,200,30,30,dsize)]
d1 = Drone(0,200,200,30,30,dsize)
print(d1.pos)
P = None
fp = [vec2(20,100),vec2(200,50),vec2(200,700),vec2(600,100)]
for p in fp:
    d1.compute_trajectory(p,steps=300)

while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
            break
    
    # calculate obstacles
    # for d in drones:
    #     d.compute_avoidance(drones)

    # check collisions
    # compute escape
                
    if d1.has_next_point() > 0 and d1.reached(d1.current_target()):
        P = d1.pop_waypoint()
        if d1.has_next_point():
            d1.steer_towards(d1.trajectory[0])
        else:
            running = False
    
    d1.move(1/60)
    screen.fill(black)
    draw_drone(d1,screen)
    pg.display.flip()
    clock.tick(60)

pg.quit()
