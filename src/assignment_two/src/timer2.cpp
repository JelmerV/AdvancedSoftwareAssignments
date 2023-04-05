#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

#define MILLISECOND 1000000

void *thread_func(void *ptr);

static void timer_handler(int signal, siginfo_t* signalInfo, void* uc);


int main() {
    pthread_t time_thread;
    
    if (pthread_create(&time_thread, NULL, thread_func, NULL) != 0) {
        perror("pthread_create error");
        exit(EXIT_FAILURE);
    }

    if (pthread_join(time_thread, NULL) != 0) {
        perror("pthread_join error");
        exit(EXIT_FAILURE);
    }

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
    signalAction.sa_sigaction = timer_handler;

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
    timerSpecification.it_interval = {1, MILLISECOND};

    // Set time specification.
    if (timer_settime(timerId, 0, &timerSpecification, NULL) != 0) {
        perror("timer_settime error");
        exit(EXIT_FAILURE);
    }

    int sig;
    while (1) {
        sigwait(&signalAction.sa_mask, &sig);
    }
}

static void timer_handler(int signal, siginfo_t* signalInfo, void* uc) {
    printf("Thread running\n");
}