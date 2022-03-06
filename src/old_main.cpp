#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <omp.h>
#include "vec2.h"
#include "drone.h"
#include "comms.h"
#include "in_out.h"

omp_lock_t writelock;

time_t t;
uint32_t iterations = 20000;

double dt = 1E-3; //seconds

double error = -1.0;

const std::vector<double> rates = {
    1E-2,
    5E-2,
    1E-1,
    5E-1,
    1.0,
    2.0,
    3.0,
    4.0,
    5.0,
    6.0,
    7.0,
    8.0,
    9.0,
    10.0,
    11.0,
    12.0,
    13.0,
    14.0,
    15.0,
    16.0}; //msg/s

int num_threads = 8;
double speed = 20.0;

int si = 0;
double rate = 1.0;
char prob = 'A';
RFsystem sys;
double l = 1000.0;

int main(int argc, char *argv[])
{
  if (argc > 1)
  {
    prob = argv[1][0];
  }
  if (argc > 2)
  {
    error = atof(argv[2]);
  }
  if (argc > 3)
  {
    l = 1000.0 * atof(argv[3]);
  }
  if (argc > 4)
  {
    speed = atof(argv[4]);
    if (speed > 100.0)
    {
      dt = 1e-4;
    }
  }

  std::cout<<"Error: "<<error<<std::endl;

  switch (prob)
  {
  case 'E':
    std::cout<<"Wi-Fi beacons"<<std::endl;
    sys = WI_FI;
    break;
  case 'C':
    std::cout<<"ADS-B\n"<<std::endl;
    sys = ADS_B;
    break;
  default:
    sys = NO_LOSS;
    std::cout<<"No loss\n"<<std::endl;
  }

  std::cout<<"Loss: "<<l<<std::endl;
  std::cout<<"Speed: "<<speed<<std::endl;

  std::vector<long> collisions(rates.size());
  for(uint64_t i =0;i<rates.size();i++){
    collisions[i] = 0;
  }
  
  omp_init_lock(&writelock);
  
  //speed is in m/s
  vec2 p1 = {500.0, 500.0};
  vec2 p2 = {1000.0, 1000.0};
  vec2 p3 = {1000.0, 0.0};

  std::cout<<"Rate:\tPcrash:"<<std::endl;
  omp_set_num_threads(num_threads);

#pragma omp parallel
  {

    for (uint32_t i = 0; i < rates.size(); i++)
    {
      std::cout<<"rate: "<<rates[i]<<std::endl;

#pragma omp for
      for (uint32_t it = 0; it < iterations; it++)
      {

        Drone d1 {0.0, 0.0, speed, 0.0, 1};
        Drone d2 {1000.0, 0.0, speed, 0.0, 1};
        
        d1.pushWaypoint(p3);
        d2.pushWaypoint(p3);

        d1.pushWaypoint(p2);
        d2.pushWaypoint(p2);
        
        d1.pushWaypoint(p1);
        d2.pushWaypoint(p1);
        

        rate = rates[i];

        bool running = true;
        double timer = 0;

        while (running)
        {

          if (timer >= 1 / rate)
          {

            if (COM_broadcast(d1.currentPosition(), d2.currentPosition(), sys, l))
            {

              d2.avoid(d1,error);
              d1.avoid(d2,error);
              
            }
            timer = 0;
          }

          d1.move(dt);
          d2.move(dt);
          
          if (d1.currentWayPoint().x == 0 && d1.currentWayPoint().y == 0)
          {
            running = false;
          }
          if (d2.currentWayPoint().x == 0 && d2.currentWayPoint().y == 0)
          {
            running = false;
          }
          
          if (d1.currentPosition().distance(d2.currentPosition()) < (d1.radius() + d2.radius())) 
          {
            omp_set_lock(&writelock);
            collisions[i] += 1;
            omp_unset_lock(&writelock);
            running = false;
          }

          timer += dt;
        }

      } //iterations
      if (collisions[i] == 0)
      {
        std::cout<<rates[i]<<std::endl;
        break;
      }
    } //rates

  } //openmp
  saveResults("results_speed_loss_avoid_1s_ttc.txt", prob, rates, collisions, error, speed, iterations, l);

  return 0;
}
