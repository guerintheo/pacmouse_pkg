#include <iostream>

#include <pigpio.h>
#include <unistd.h>


#include "rotary_encoder.hpp"

/*

REQUIRES

A rotary encoder contacts A and B connected to separate gpios and
the common contact connected to Pi ground.

TO BUILD

g++ -o rot_enc_cpp test_rotary_encoder.cpp rotary_encoder.cpp -lpigpio -lrt

TO RUN

sudo ./rot_enc_cpp

*/

// These are all GPIO numbers
#define ENCODER_A 23
#define ENCODER_B 24

#define MOT_GPIO 12
#define PWM_DUTY 255

void callback(int way)
{
   static int pos = 0;

   pos += way;

   std::cout << "pos=" << pos << std::endl;
}

int main(int argc, char *argv[])
{
   if (gpioInitialise() < 0) return 1;

   re_decoder dec(ENCODER_A, ENCODER_B, callback);
   // red_ecoder dec = red_decoder()
   
   usleep(1000000);

	gpioPWM(MOT_GPIO, PWM_DUTY);
	usleep(1000000);
 	gpioPWM(MOT_GPIO, 0);
	usleep(1000000);
	gpioPWM(MOT_GPIO, -PWM_DUTY);

   usleep(100000000);

   dec.re_cancel();

   gpioTerminate();
}

