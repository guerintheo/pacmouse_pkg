#include <pigpio.h>

int main() {
	gpioSetMode(12, PI_INPUT);
	return 0;
}