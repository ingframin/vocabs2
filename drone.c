#include "drone.h"
#include<stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif
#include "vec2.h"

static uint32_t ids =0;

const Obstacle compute_obstacle(Drone *d1, Drone* d2)
{
	// Minkowski addition
	double r = d1->size + d2->size;
	vec2 center;
	//Computing tangent lines to circle passing through the point self.position
	double dx = center.x - d2->position.x;
	double a = dx * dx - r * r;
	double b = 2 * dx * (center.y - d2->position.y);
	double c = (d2->position.y - center.y) * (d2->position.y - center.y) - r * r;
	double Delta = b * b - 4 * a * c;

	//Angular coefficient
	double m1 = (-b + sqrt(Delta)) / (2 * a);
	double m2 = (-b - sqrt(Delta)) / (2 * a);
	//Intersection with y axis
	double q1 = center.y - m1 * center.x;
	double q2 = center.y - m2 * center.x;

	//(xt1,yt1) - first tangent point.
	double a1 = 1 + m1 * m1;
	double b1 = 2 * m1 * q1 - 2 * d2->position.x - m1 * 2 * d2->position.y;

	double xt1 = (-b1) / (2 * a1);
	double yt1 = m1 * xt1 + q1;

	//(xt2,yt2) - Second tangent point
	double a2 = 1 + m2 * m2;
	double b2 = 2 * m2 * q2 - 2 * d2->position.x - m2 * 2 * d2->position.y;

	double xt2 = (-b2) / (2 * a2);
	double yt2 = m2 * xt2 + q2;

	//Construct obstacle
	Obstacle o;
	o.position = d2->position;
	o.radius = r;
	o.T1.x = xt1;
  	o.T1.y = yt1;
	o.T2.x = xt2;
  	o.T2.y = yt2;

	return o;
}

barycoords barycentric(vec2 A, vec2 B, vec2 C, vec2 P){
  	barycoords bc;
  	bc.gamma = ((A.y - B.y) * P.x + (B.x - A.x) * P.y + A.x * B.y - B.x * A.y) /
				((A.y - B.y) * C.x + (B.x - A.x) * C.y + A.x * B.y - B.x * A.y);
	bc.beta = ((A.y - C.y) * P.x + (C.x - A.x) * P.y + A.x * C.y - C.x * A.y) /
			   ((A.y - C.y) * B.x + (C.x - A.x) * B.y + A.x * C.y - C.x * A.y);
	bc.alpha = 1 - bc.beta - bc.gamma;
  	return bc;

}
    
Drone DR_newDrone(double x, double y,double vx,double vy, double size){
	Drone d;
	d.id = ids;
	ids++;
	d.position.x = x;
	d.position.y = y;
	d.speed.x = vx;
	d.speed.y = vy;
	d.waypoints = malloc(2*sizeof(vec2));
	d.wp_len = 2;
	d.curr_wp = 0;
	d.size = size;
	return d;
}

void DR_move(Drone* d, double dt){
	if(d->curr_wp < 0){
		return;
	}
	if(v2_distance(&d->position,&d->waypoints[d->curr_wp]) < d->size){
		DR_pop_waypoint(d);
	}
	
	DR_goto(d,d->waypoints[d->curr_wp]);
	vec2 dP = v2_prodK(&d->speed,dt);
	d->position = v2_add(&d->position,&dP);

}

void DR_goto(Drone* d, vec2 waypoint){
  
  vec2 dir = v2_norm(&d->speed);
  
  vec2 dirp = v2_sub(&waypoint, &d->position);
  
  dirp = v2_norm(&dirp);
  
  double C = v2_dot(&dir,&dirp);
  
  if(C > 1.0){
    C=1.0;
  }
  else if(C < -1.0){
    C=-1.0;
  }

  double angle = -acos(C);
  //steer
  
  d->speed = v2_rotate(&d->speed,angle);

}

bool DR_collision(Drone* d1, Drone* d2){
	Obstacle o = compute_obstacle(d1,d2);
	vec2 dif = v2_sub(&d1->speed,&d2->speed);
	vec2 ds = v2_add(&dif,&d1->position);
	barycoords bc = barycentric(o.position,o.T1,o.T2,ds);
	if(bc.alpha>0 && bc.beta >0 && bc.gamma>0){
		return true;
	}

    return false;

}
void DR_avoid(Drone* d, Drone* d2){
	if(DR_collision(d,d2)){
		vec2 escape = v2_rotate(&d->speed,M_PI/2);
		escape = v2_norm(&escape);
		escape = v2_prodK(&escape,2*(d->size+d2->size));
		escape = v2_add(&escape,&d->position);
		DR_push_waypoint(d,escape);
		
	}
}
void DR_push_waypoint(Drone* d, vec2 wp){
	
	if(d->curr_wp==(d->wp_len-1)){
		d->wp_len = (d->wp_len+(d->wp_len>>1));
		d->waypoints = realloc(d->waypoints,(d->wp_len+(d->wp_len>>1))*sizeof(vec2));	 
	}
	d->curr_wp += 1;
	d->waypoints[d->curr_wp] = wp;
}
void DR_pop_waypoint(Drone* d){
	d->curr_wp -= 1;
}

void DR_freeDrone(Drone*d){
	free(d->waypoints);
}
