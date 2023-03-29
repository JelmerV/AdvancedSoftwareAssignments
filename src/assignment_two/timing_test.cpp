#include <iostream>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <cmath>

static bool run_thread = true;
static int count = 0;
static int missed_deadlines = 0;


static double LOOP_RUN_TIME = 10;   // seconds to test
static double LOOP_INTERVAL = 1;    // ms


void *loop(void *ptr) {
  struct timespec next_time, current_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  double sleep_time;

  while (run_thread) {
    count++;

    // Do some computational work
    for (double i = 0; i < 10000; ++i) {
      double x = sin(i);
    }

    // Calculate next period
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    int timesteps = 0;
    do {
      timesteps++;
      next_time.tv_nsec += LOOP_INTERVAL * 1.0e6;
      if (next_time.tv_nsec >= 1e9) {
        next_time.tv_nsec -= 1e9;
        next_time.tv_sec++;
      }
      sleep_time = (next_time.tv_sec - current_time.tv_sec) * 1e9 + (next_time.tv_nsec - current_time.tv_nsec);
    } while (sleep_time < 0); // If deadline missed, increment time

    if (timesteps > 1) {
      missed_deadlines++;
    }

    // Sleep until next period
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }

  return NULL;
}

int main(int argc, char * argv[])
{
  struct timespec loop_start_time, loop_end_time;
  double elapsed_time;

  pthread_t t;

  printf("starting loop, running for %f seconds\n", LOOP_RUN_TIME);

  // run thread_func within the time_thread thread
  if (pthread_create(&t, NULL, loop, NULL) != 0) {
      perror("pthread_create error");
      exit(EXIT_FAILURE);
  }
  clock_gettime(CLOCK_MONOTONIC, &loop_start_time);

  // Wait for 5 seconds and stop loop
  sleep(LOOP_RUN_TIME);
  run_thread = false;

  // Wait for loop to finish
  if (pthread_join(t, NULL) != 0) {
      perror("pthread_join error");
      exit(EXIT_FAILURE);
  }
  clock_gettime(CLOCK_MONOTONIC, &loop_end_time);

  // Compute elapsed time
  elapsed_time = loop_end_time.tv_sec - loop_start_time.tv_sec + (loop_end_time.tv_nsec - loop_start_time.tv_nsec) / 1.0e9;
  double average_time = 1e3 * (elapsed_time/count);

  // calulate and print average time
  printf("\nLoop ran %d times in %f seconds\n", count, elapsed_time);
  printf("average time between loops: %fms, targeting %fms\n", average_time, LOOP_INTERVAL);
  printf("missed deadlines: %d\n", missed_deadlines);

  return 0;
}