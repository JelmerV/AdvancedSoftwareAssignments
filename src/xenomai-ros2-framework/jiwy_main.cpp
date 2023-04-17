#include <stdio.h>
#include <unistd.h>
#include <iterator>
#include <signal.h>
#include <vector>
#include <sys/syscall.h>

#include "framework/multiCommClass.h"
#include "framework/runnableClass.h"
#include "framework/superThread.h"
#include "framework/icoCommClass.h"

#include "ControllerPan/ControllerPan.h"

volatile bool exitbool = false;

void exit_handler(int s)
{
    printf("Caught signal %d\n", s);
    exitbool = true;
}

void ReadConvert(const double* src, double* dst) {
    static double lastKnownGoodValue = 0;

    if (src[2] == 0) {
        dst[2] = (src[2]/0.8)*2047;
        lastKnownGoodValue = dst[2];
    } else {
        dst[2] = lastKnownGoodValue;
    }
}

void WriteConvert(const double* src, double* dst) {
    dst[3] = 1.6*(src[3]/16383.0)-0.8;
}

int main()
{
    // CREATE CNTRL-C HANDLER
    signal(SIGINT, exit_handler);

    printf("Press Ctrl-C to stop program\n"); // Note: this will
                                              // not kill the program; just jump out of the wait loop. Hence,
                                              // you can still do proper clean-up. You are free to alter the
                                              // way of determining when to stop (e.g., run for a fixed time).

    // CONFIGURE, CREATE AND START THREADS HERE

    // CREATE PARAM AND WRAPPER FOR CONTROLLER
    int ico_uParam[] = {0, 0, 0, 1, -1, -1, -1, -1};
    int ico_yParam[] = {0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1};

    int xddp_uParam_Setpoint[] = {1};
    int xddp_yParam_Logging[] = {0};

    auto icoComm = new IcoComm(ico_uParam, ico_yParam);
    icoComm->setReadConvertFcn(&ReadConvert);
    icoComm->setWriteConvertFcn(&WriteConvert);
    frameworkComm *controller_uPorts[] = {
        new XDDPComm(10, -1, 1, xddp_uParam_Setpoint),
        icoComm
    };
    frameworkComm *controller_yPorts[] = {
        new XDDPComm(25, -1, 1, xddp_yParam_Logging),
        icoComm
    };

    ControllerPan *controller_class = new ControllerPan;
    runnable *controller_runnable = new wrapper<ControllerPan>(
        controller_class, controller_uPorts, controller_yPorts, 2, 2);
    controller_runnable->setSize(2, 2);

    xenoThread controllerClass(controller_runnable);
    controllerClass.init(1000000, 98, 0);
    controllerClass.enableLogging(true, 26);

    // START THREADS
    controllerClass.start(" controller ");

    // WAIT FOR CNTRL-C
    timespec t = {.tv_sec = 0, .tv_nsec = 100000000}; // 1/10 second

    while (!exitbool)
    {
        // Let the threads do the real work
        nanosleep(&t, NULL);
        // Wait for Ctrl-C to exit
    }
    printf("Ctrl-C was pressed: Stopping gracefully...\n");

    // CLEANUP HERE
    controllerClass.stopThread();
    controllerClass.~xenoThread();
    controller_runnable->~runnable();

    return 0;
}