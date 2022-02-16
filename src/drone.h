#ifndef DRONE_H
#define DRONE_H
#include "vec2.h"
#include <cstdint>
#include <vector>
#include <numbers>



class Drone
{
public:
  Drone(double x, double y, double vx, double vy, double size);
  Drone(const Drone& d2);
  //move the drone by speed x time delta
  void move(double dt);

  //Steer towards next waypoint
  void steer(vec2 waypoint);

  //Are the drones on a collision route?
  bool collision(const Drone& d2) const;
  
  //Compute avoidance maneuver and add escape waypoint
  void avoid(const Drone& d2, double error);

  //Waypoints are stacked (LIFO) push adds a waypoint on top of the stack
  //Increase the waypoint array size if needed
  void pushWaypoint(vec2 wp);
  //pop removes the top waypoints (but it does not shrink the waypoint array)
  void popWaypoint();
  vec2 currentWayPoint()const{
    return waypoints.back();
  }
  vec2 currentPosition()const {return position;}
  vec2 currentVelocity()const{return velocity;}
  double radius() const {return size;}

private:
  uint32_t id;       //Unique ID
  vec2 position;     // current position
  vec2 velocity;        //current speed
    /*I might consider making the flight plan a separate object*/
  std::vector<vec2> waypoints;      //flight plan (array of waypoints that rescales automagically when adding new waypoints)
  double size;          //Physical size of the drone

};



#endif