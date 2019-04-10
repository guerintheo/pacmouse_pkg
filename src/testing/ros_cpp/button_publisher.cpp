#include "ros/ros.h"
#include <pigpio.h>
#include <iostream>
#include <std_msgs/Int8.h>

using namespace std;

class Buttons {
public:
	Buttons(int p[4], ros::NodeHandle n) {

		char channel[20];
		
		for (int i = 0; i < 4; i++) {
			int pin = p[i];

			// setup a publisher
			sprintf(channel, "/pacmouse/buttons/%d", i+1);
			ros::Publisher publisher = n.advertise<std_msgs::Int8>(channel, 1);

			// configure the gpio
			gpioSetMode(pin, PI_INPUT);
			gpioSetPullUpDown(pin, PI_PUD_UP);
			gpioSetAlertFuncEx(pin, callback, &publisher);
		}
	};

	static void callback(int gpio, int level, uint32_t tick, void *user);
};


void Buttons::callback(int gpio, int level, uint32_t tick, void *user) {
   ros::Publisher *publisher = (ros::Publisher *)user;
   std_msgs::Int8 msg;
   msg.data = level;
   publisher->publish(msg);
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

