#include "ros/ros.h"
#include <pigpio.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <unistd.h>

using namespace std;

class Buttons {
public:
	ros::Publisher publishers[4];
	int pins[4];
	bool old_levels[4];
	bool levels[4];

	Buttons(int p[4], ros::NodeHandle n) {

		char channel[20];
		
		for (int i = 0; i < 4; i++) {
			int pin = pins[i] = p[i];

			// setup a publisher
			sprintf(channel, "/pacmouse/buttons/%d", i+1);
			publishers[i] = n.advertise<std_msgs::Bool>(channel, 1);

			// configure the gpio
			gpioSetMode(pin, PI_INPUT);
			gpioSetPullUpDown(pin, PI_PUD_DOWN);
		}
	};

	void publish_changes();
};

void Buttons::publish_changes() {
   std_msgs::Bool msg;

	for (int i = 0; i < 4; i++) {
		levels[i] = gpioRead(pins[i]);
		if (old_levels[i] != levels[i]) {
			msg.data = old_levels[i] = levels[i];
	  		publishers[i].publish(msg);
		}
	}
}

// int main(int argc, char **argv)
// {
//   if (gpioInitialise() < 0) {
//     ROS_INFO("Could not initialize GPIOs. I'm borked.");
//     return 1;
//   }
//   ros::init(argc, argv, "button_publisher");
//   ros::NodeHandle n;

//   int p[] = {7, 13, 16, 12};
//   Buttons b = Buttons(p, n);

//   while (ros::ok()) {  
//   	b.publish_changes();
//   	usleep(100000000);
//   }
//   return 0;
// }
