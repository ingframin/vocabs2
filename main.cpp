#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <omp.h>
#include "vec2.h"
#include "drone.h"
#include "comms.h"
omp_lock_t writelock;

time_t t;
uint32_t iterations = 20000;

double dt = 1E-3; //seconds

double error = -1.0;

double rates[] = {
    1E-2,
    2.5E-2,
    5E-2,
    7.5E-2,
    1E-1,
    2.5E-1,
    5E-1,
    7.5E-1,
    1.0,
    1.25,
    1.5,
    1.75,
    2.0,
    2.25,
    2.5,
    2.75,
    3.0,
    3.25,
    3.5,
    3.75,
    4.0,
    4.25,
    4.5,
    4.75,
    5.0,
    5.25,
    5.5,
    5.75,
    6.0,
    6.25,
    6.5,
    6.75,
    7.0,
    7.25,
    7.5,
    7.75,
    8.0,
    8.25,
    8.5,
    8.75,
    9.0,
    9.25,
    9.5,
    9.75,
    10.0,
    10.25,
    10.5,
    10.75,
    11.0,
    11.25,
    11.5,
    11.75,
    12.0,
    12.25,
    12.5,
    12.75,
    13.0,
    13.25,
    13.5,
    13.75,
    14.0,
    14.25,
    14.5,
    14.75,
    15.0,
    15.25,
    15.5,
    15.75,
    16.0}; //msg/s

int num_threads = 8;
double speed = 20.0;
uint32_t len_rates = sizeof(rates) / sizeof(double);
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

  printf("Error: %.3f\n", error);

  switch (prob)
  {
  case 'E':
    printf("Wi-Fi beacons\n");
    sys = WI_FI;
    break;
  case 'C':
    printf("ADS-B\n");
    sys = ADS_B;
    break;
  default:
    sys = NO_LOSS;
    printf("No loss\n");
  }

  printf("Loss: %.3f\n", l);
  printf("Speed: %.3f\n", speed);

  double collisions[len_rates];
  for (uint32_t k = 0; k < len_rates; k++)
  {
    collisions[k] = 0.0;
  }

  omp_init_lock(&writelock);
  t = time(NULL);
  srand(t);

  //speed is in m/s

  vec2 p1 = {500.0, 500.0};
  vec2 p2 = {1000.0, 1000.0};
  

  printf("Rate:\tPcrash:\n");
  omp_set_num_threads(num_threads);

#pragma omp parallel
  {

    for (uint32_t i = 0; i < len_rates; i++)
    {
      printf("rate: %.2f \n", rates[i]);

#pragma omp for
      for (uint32_t it = 0; it < iterations; it++)
      {

        Drone d1 = DR_newDrone(0.0, 0.0, speed, 0.0, 1);
        Drone d2 = DR_newDrone(1000.0, 0.0, speed, 0.0, 1);
        
        DR_push_waypoint(&d1, p2);
        DR_push_waypoint(&d2, p2);
        
        DR_push_waypoint(&d1, p1);
        DR_push_waypoint(&d2, p1);
        

        rate = rates[i];

        bool running = true;
        double timer = 0;

        while (running)
        {

          if (timer >= 1 / rate)
          {

            if (COM_broadcast(d1.position, d2.position, sys, l))
            {

              DR_avoid(&d2, &d1, error);
              DR_avoid(&d1, &d2, error);
              
            }
            timer = 0;
          }

          DR_move(&d1, dt);
          DR_move(&d2, dt);
          
          if (d1.waypoints.back().x == 0 && d1.waypoints.back().y == 0)
          {
            running = false;
          }
          if (d2.waypoints.back().x == 0 && d2.waypoints.back().y == 0)
          {
            running = false;
          }
          
          if (v2_distance(d1.position, d2.position) < (d1.size + d2.size)) 
          {
            omp_set_lock(&writelock);
            collisions[i] += 1;
            omp_unset_lock(&writelock);
            running = false;
          }

          timer += dt;
        }

      } //iterations
      if (collisions[i] == 0.0)
      {
        printf("%.3f\n", rates[i]);
        break;
      }
    } //rates

  } //openmp

  FILE *results = fopen("results_speed_loss_avoid_1s_ttc.txt", "a");
  fprintf(results, "Error: %.3f\n", error);
  fprintf(results, "Loss: %.3f\n", l);
  fprintf(results, "Speed: %.3f\n", speed);
  switch (prob)
  {
  case 'E':
    fprintf(results, "Wi-Fi beacons\n");
    break;
  case 'C':
    fprintf(results, "ADS-B\n");
    break;
  default:
    fprintf(results, "No loss\n");
  }

  for (uint32_t k = 0; k < len_rates; k++)
  {
    printf("%.3f\t%.6f\n", rates[k], collisions[k] / iterations);
    fprintf(results, "%.3f\t%.10f\n", rates[k], collisions[k] / iterations);
  }
  fclose(results);

  return 0;
}
