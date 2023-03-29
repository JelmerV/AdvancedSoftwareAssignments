#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>

#define MILLISECOND 1000000

void *thread_func(void *ptr);

static void timer_handler(int signal, siginfo_t* signalInfo, void* uc);

static int count = 0;
static bool run_thread = true;


int main() {
    pthread_t time_thread;
    
    // run thread_func within the time_thread thread
    if (pthread_create(&time_thread, NULL, thread_func, NULL) != 0) {
        perror("pthread_create error");
        exit(EXIT_FAILURE);
    }

    printf("starting timer and waiting 5 seconds");
    // wait for 5 seconds
    sleep(5);

    // stop the time_thread
    run_thread = false;

    // wait for the time_thread to finish
    if (pthread_join(time_thread, NULL) != 0) {
        perror("pthread_join error");
        exit(EXIT_FAILURE);
    }

    // print the number of interrupts
    printf("Number of interrupts: %d\n", count);
    printf("average time per interrupt: %f\n", 5.0 / count);

    return 0;
}


void *thread_func(void *ptr) {
    timer_t timerId = 0;

    // Define signal event.
    struct sigevent signalEvent;
    signalEvent.sigev_notify = SIGEV_SIGNAL;
    signalEvent.sigev_signo = SIGUSR1;

    // Define signal action.
    struct sigaction signalAction;
    signalAction.sa_flags = SA_SIGINFO;
    signalAction.sa_sigaction = timer_handler;  //call timer_handler when interrupt is triggered

    // Initialise signal.
    sigemptyset(&signalAction.sa_mask);

    // Check if the action has been setup properly.
    if (sigaction(SIGUSR1, &signalAction, NULL) != 0) {
        perror("sigaction error");
        exit(EXIT_FAILURE);
    }

    // Create the timer.
    if (timer_create(CLOCK_MONOTONIC, &signalEvent, &timerId) != 0) {
        perror("timer_create error");
        exit(EXIT_FAILURE);
    }

    // Create the timer specification.
    struct itimerspec timerSpecification;
    timerSpecification.it_value    = {0, MILLISECOND};
    timerSpecification.it_interval = {0, MILLISECOND};

    // Set time specification.
    if (timer_settime(timerId, 0, &timerSpecification, NULL) != 0) {
        perror("timer_settime error");
        exit(EXIT_FAILURE);
    }

    int sig;
    while (run_thread) {
        sigwait(&signalAction.sa_mask, &sig);
    }
}

static void timer_handler(int signal, siginfo_t* signalInfo, void* uc) {
    // measure time since last interrupt
    static struct timespec prev_time = {0, 0};
    struct timespec curr_time;
    clock_gettime(CLOCK_MONOTONIC, &curr_time);

    double elapsed_time = curr_time.tv_sec - prev_time.tv_sec +
      (curr_time.tv_nsec - prev_time.tv_nsec) / 1e9;

    prev_time = curr_time;

    printf("Elapsed time: %f\n", elapsed_time);

    // Do some computational work
    for (int i = 0; i < 10000; ++i) {
      double x = sin(i);
    }
    count++;
}