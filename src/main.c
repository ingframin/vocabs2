#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <omp.h>
#include "vec2.h"
#include "flightplan.h"
#include "drone.h"
#include "comms.h"
#include "main.h"
#ifdef TEST
#include "test_vec2.h"
#include "test_flightplan.h"

int main(int argc, char** argv){
  bool tests[] = {
    test_v2_mod(),
    test_v2_rotateHalfPI(),
    test_v2_rotate(),
    test_newFlightPlan(),
    test_push_waypoint(),
    test_pop_waypoint()
  };
  for(int i =0; i<6;i++){
    printf("T%d pass: %s\n",i,tests[i] ? "true" : "false");
  }
  

  return 0;
}
#else
omp_lock_t writelock;

int main(int argc, char *argv[])
{

  
  parseArguments(argc,argv);
  
  double collisions[len_rates];
  for (uint32_t k = 0; k < len_rates; k++)
  {
    collisions[k] = 0.0;
  }
  omp_init_lock(&writelock);
  t = time(NULL);
  srand(t);

  // speed is in m/s

  vec2 p1 = {500.0, 500.0};
  vec2 p2 = {1000.0, 1000.0};
  

  printf("Rate:\tPcrash:\n");
  omp_set_num_threads(num_threads);
//The simulation code should go into a separate function
#pragma omp parallel
  {

    for (uint32_t i = 0; i < len_rates; i++)
    {
      printf("rate: %.2f \n", 1000.0/rates[i]);

#pragma omp for
      for (uint32_t it = 0; it < iterations; it++)
      {
        //Should Drone become a reference type?
        Drone d1 = DR_newDrone(0.0, 0.0, speed, 0.0, 1);
        Drone d2 = DR_newDrone(1000.0, 0.0, speed, 0.0, 1);
        
        FP_push_waypoint(d1.fp, p2);
        FP_push_waypoint(d2.fp, p2);
        
        FP_push_waypoint(d1.fp, p1);
        FP_push_waypoint(d2.fp, p1);
        

        rate = rates[i];
        //Timer should be part of the simulation object
        bool running = true;
        uint64_t timer = 0;

        while (running)
        {

          if (timer >= rate)
          {

            // if (COM_broadcast(d1.position, d2.position, sys, l))
            if (COM_broadcast_Pint(0.5,0.5,0.0))
            {
              
              DR_avoid(&d2, &d1, error);
              DR_avoid(&d1, &d2, error);
              
            }
            timer = 0;
          }

          DR_move(&d1, dt);
          DR_move(&d2, dt);
          
          
          if (d1.fp->empty)
          {
            running = false;
          }

          if (d2.fp->empty)
          {
            running = false;
          }
          //This should be improved for better collision detection
          if (v2_distance(d1.position, d2.position) < (d1.size + d2.size)) 
          {
            omp_set_lock(&writelock);
            collisions[i] += 1;
            omp_unset_lock(&writelock);
            running = false;
          }

          timer += 1;
        }//while
        // printf("Iter: %d",it);
        //Fix for memory leak: flight plan within drones was allocated with malloc and never freed.
        DR_freeDrone(&d1);
        DR_freeDrone(&d2);
      } //iterations
      if (collisions[i] == 0.0)
      {
        printf("%.3f\n", 1000.0/rate);
        break;
      }
    } //rates

  } //openmp

  
  saveResults("pinterf_0.0.txt", collisions);
  return 0;
}
#endif