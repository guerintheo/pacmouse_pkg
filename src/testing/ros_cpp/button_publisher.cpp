#include "ros/ros.h"
#include <pigpio.h>
#include <iostream>
#include <std_msgs/Int8.h>

using namespace std;

class Buttons {
	int pins[4];
	ros::Publisher publishers[4];
public:
	Buttons(int p[4], ros::NodeHandle n) {

		char channel[20];
		
		for (int i = 0; i < 4; i++) {
			int pin = pins[i] = p[i]; // copy the pin into the pins array

			// configure the gpio
			gpioSetMode(pin, PI_INPUT);
			gpioSetPullUpDown(pin, PI_PUD_UP);
			gpioSetAlertFuncEx(pin, callback, this);

			// and setup a publisher
			sprintf(channel, "/pacmouse/buttons/%d", i+1);
			publishers[i] = n.advertise<std_msgs::Int8>(channel, 1);
		}
	};

	static void callback(int gpio, int level, uint32_t tick, void *user);
};


void Buttons::callback(int gpio, int level, uint32_t tick, void *user) {

   // pass the calling instance of the button class through so we can access its attributes
   Buttons *b = (Buttons *) user;

   // get the index of the gpio so we can look up the corresponding publisher
   int i = std::distance(b->pins, std::find(b->pins, b->pins + 4, gpio));
   printf("pin %d is %d\n", i, level);
   std_msgs::Int8 msg;
   msg.data = level;
   b->publishers[i].publish(msg);
}

int main(int argc, char **argv)
{
  if (gpioInitialise() < 0) {
    ROS_INFO("Could not initialize GPIOs. I'm borked.");
    return 1;
  }
  ros::init(argc, argv, "button_publisher");
  ros::NodeHandle n;
  int p[] = {7, 13, 16, 12};
  Buttons b = Buttons(p, n);
  ros::spin();
  return 0;
}

