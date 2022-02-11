#ifndef DRONE_H
#define DRONE_H
#include "vec2.h"
#include <stdint.h>
#include <stdbool.h>
#include <vector>
#include <numbers>
#include <random>
#include <memory>

//Struct to keep barycentric coordinates
struct Barycoords
{
  double alpha;
  double beta;
  double gamma;
};

struct Obstacle
{
  //radius
  double radius;
  //center
  vec2 position;
  //tangent points
  vec2 T1;
  vec2 T2;
};

class Drone
{
public:
  Drone(double x, double y, double vx, double vy, double size, double error);
  Drone();
  Drone(const Drone& d);
  //move the drone by speed x time delta
  void move(double dt);

  //Steer towards next waypoint
  void steer(vec2 waypoint);

  //Are the drones on a collision route?
  bool collision(Drone& d2);
  
  //Compute avoidance maneuver and add escape waypoint
  void avoid(Drone& d2);
  
  //Instead of computing an avoidance maneuver waits until no collision is imminent
  void stopAndWait(Drone& d2);

  //Waypoints are stacked (LIFO) push adds a waypoint on top of the stack
  //Increase the waypoint array size if needed
  void pushWaypoint(vec2 wp);
  //pop removes the top waypoints (but it does not shrink the waypoint array)
  void popWaypoint();
  vec2 currentWayPoint(){
    return waypoints.back();
  }
  vec2 currentPosition(){return position;}
  vec2 currentVelocity(){return velocity;}
  double radius(){return size;}

private:
  uint32_t id;       //Unique ID
  vec2 position;     // current position
  vec2 velocity;        //current speed
  double _speed_mod; //speed module
  /*I might consider making the flight plan a separate object*/
  std::vector<vec2> waypoints;      //flight plan (array of waypoints that rescales automagically when adding new waypoints)
  double size;          //Physical size of the drone
  std::unique_ptr<std::random_device> rng;
  std::normal_distribution<double> gaussian;
  double error;
};



#endif
