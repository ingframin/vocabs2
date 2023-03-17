#ifndef MAIN_H
#define MAIN_H
#ifdef __unix
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),(mode)))==NULL
#endif
time_t t;
uint32_t iterations = 10000;

double dt = 1E-3; //seconds

/*This is supposed to be the positional error in m
It's an ugly solution, it should be corrected.
Maybe it makes more sense to have an error_x and error_y
or make it a struct.
*/
double error = 0;
/*These are not rates but timings in ms.
The timings are not integers but the timer is.
This should be changed or even calculated at runtime.
*/
double rates[] = 
{
  
  2000.000,
  1000.000,
  500.000,
  333.333,
  250.000,
  200.000,
  166.667,
  142.857,
  125.000,
  111.111,
  100.000,
  90.909,
  83.333,
  76.923,
  71.429,
  66.667,
  62.500,
  58.824,
  55.556,
  52.632,
  50.000
}; //msg/s
    
int num_threads = 8;
double speed = 20.0;
uint32_t len_rates = sizeof(rates) / sizeof(double);
int si = 0;
uint64_t rate = 1;
char prob = 'A';
RFsystem sys;
//loss probability x1000
double l = 1000.0;

void parseArguments(int argc, char *argv[]){
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

}

/*Maybe I should add some error checking? :-/*/
void saveResults(const char* filename, double collisions[]){
  FILE *results;
  fopen_s(&results,filename, "a");
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
    printf("%.3f\t%.6f\n", 1000.0/rates[k], collisions[k] / iterations);
    fprintf(results, "%.3f\t%.10f\n", 1000.0/rates[k], collisions[k] / iterations);
  }
  fclose(results);
}

#endif