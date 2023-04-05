#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

// Define loop variables.
static bool run_thread = true;
static double LOOP_RUN_TIME = 10;   // seconds to test
static double LOOP_INTERVAL = 1;    // ms

// Define loop statistic variables.
static int count = 0;
static int count_too_late = 0;
static int missed_deadlines = 0;

// Define variables for jitter.
#define NUM_BINS 100
static int jitter_bins[NUM_BINS] = {0};

// Thread function.
void *loop(void *ptr) {
    // Define time specification variables for sleeping.
    struct timespec next_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    double sleep_time;

    while (run_thread) {
        count++;

        // Calculate timing error.
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double timing_error = (current_time.tv_sec - next_time.tv_sec) * 1e9 + (current_time.tv_nsec - next_time.tv_nsec);
        int bin = (int) (timing_error*1e-4) + NUM_BINS/2;
        bin = fmin(fmax(bin, 0), NUM_BINS-1);
        jitter_bins[bin]++;

        // Code to create computational load within the loop.
        for (double i = 0; i < 5500; ++i) {
            double x = sin(i);
        }

        // Calculate the next period.
        int timesteps = 0;
        do {
            timesteps++;
            next_time.tv_nsec += LOOP_INTERVAL * 1.0e6;

            // Fix time specification if necessary for the correct amount of seconds.
            if (next_time.tv_nsec >= 1e9) {
                next_time.tv_nsec -= 1e9;
                next_time.tv_sec++;
            }

            clock_gettime(CLOCK_MONOTONIC, &current_time);
            sleep_time = (next_time.tv_sec - current_time.tv_sec) * 1e9 + (next_time.tv_nsec - current_time.tv_nsec);
        } while (sleep_time < 0); // If deadline was missed, increment time.

        // Check for missed deadlines.
        if (timesteps > 1) {
            missed_deadlines += timesteps - 1;
            count_too_late++;
        }

        // Sleep until next period.
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }

    return NULL;
}

int main(int argc, char * argv[])
{
    // Define time related variables.
    struct timespec loop_start_time, loop_end_time;
    double elapsed_time;

    // Define thread.
    pthread_t time_thread;

    // Print start message.
    printf("starting loop, running for %f seconds\n", LOOP_RUN_TIME);

    // Run 'loop' function within the 'time_thread' thread.
    if (pthread_create(&time_thread, NULL, loop, NULL) != 0) {
        perror("pthread_create error");
        exit(EXIT_FAILURE);
    }
    clock_gettime(CLOCK_MONOTONIC, &loop_start_time);

    // Wait for 5 seconds and stop the loop.
    sleep(LOOP_RUN_TIME);
    run_thread = false;

    // Wait for loop to finish.
    if (pthread_join(time_thread, NULL) != 0) {
        perror("pthread_join error");
        exit(EXIT_FAILURE);
    }
    clock_gettime(CLOCK_MONOTONIC, &loop_end_time);

    // Compute elapsed time.
    elapsed_time = loop_end_time.tv_sec - loop_start_time.tv_sec + (loop_end_time.tv_nsec - loop_start_time.tv_nsec) / 1.0e9;
    double average_time = 1e3 * (elapsed_time/count);

    // Calulate and print average time and other relevant data.
    printf("\nLoop ran %d times in %f seconds\n", count, elapsed_time);
    printf("average time between loops: %fms, targeting %fms\n", average_time, LOOP_INTERVAL);
    printf("total missed deadlines: %d, split over %d loops.\n", missed_deadlines, count_too_late);

    // Print histogram.
    printf("\njitter histogram (ms): \n");
    for (int i = 0; i < NUM_BINS; i++) {
        float ms = (i - NUM_BINS/2);
        ms = ms/100;
        printf("%.3fms: \t %d\n", ms, jitter_bins[i]);
    }
      
    return 0;
}