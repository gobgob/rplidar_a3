// example of use of the lib piGPIO
// add to the MAKEFILE: -lpigpio -lrt

#include <pigpio.h>
gpioInitialise();

gpioSetMode(12, PI_INPUT);
gpioSetMode(12, PI_OUTPUT);
gpioSetMode(12, PI_ALT0);

// pin, frequency, duty-cycle
gpioHardwarePWM(12, 25000, 0);

gpioTerminate();
