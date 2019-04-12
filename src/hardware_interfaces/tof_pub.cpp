#include <vl6180_pi/vl6180_pi.h>
#include "ros/ros.h"
#include <stdio.h>
#include <wiringPi.h>
#include <unistd.h>
#include "pacmouse_pkg/Lidars.h"

//for some reason the linker wasn't linking to the right place, so i run
// cp /usr/local/lib/libvl6180_pi.* ~/ros_catkin_ws/devel/lib/.

int main(int argc, char **argv){

	ros::init(argc, argv, "lidar_publisher");
	ros::NodeHandle n;

	int num_lidars;
	n.getParam("/pacmouse/params/num_lidars", num_lidars);


	int power_pins[num_lidars];
	n.getParam("/pacmouse/params/lidar_pins", power_pins);

	setbuf(stdout, NULL);
	
	// int power_pins[] = {7, 19, 11, 23, 29}; // 6 and 7 are 31 and 37
	// removed lidar 4,5 on pin 23,29 at address 0x44,0x45
	char addresses[] = {0x40, 0x41, 0x42, 0x43, 0x44};
	vl6180 handles[num_lidars];

	wiringPiSetupPhys();
	
	for (int i=0; i < num_lidars; i++) {
		pinMode(power_pins[i], OUTPUT);
		digitalWrite(power_pins[i], LOW);
	}
	printf("VL6180s LOW\n");
	sleep(1);



	for (int i=0; i < num_lidars; i++) {
		digitalWrite(power_pins[i], HIGH);
		sleep(0.5);
		handles[i] = vl6180_initialise(1);
		vl6180_change_addr(handles[i], addresses[i]);
		int dist = get_distance(handles[i]);
		printf("test dist: %d\n", dist);
		if (handles[i]<=0) {
			printf("Failed %x\n", addresses[i]);
			return 1;
		}
		else {
			printf("Init %x\n", addresses[i]);
			set_scaling(handles[i],1);
		}
	}



	ros::Publisher lidar_publisher = n.advertise<pacmouse_pkg::Lidars>("/pacmouse/lidars", 1);
	
	pacmouse_pkg::Lidars msg;

	while (ros::ok()) {  
		for (int i=0; i < num_lidars; i++) {
			msg.dists[i] = get_distance(handles[i]);
			printf("%d\t", msg.dists[i]);
		}
		printf("\n");
		lidar_publisher.publish(msg);
	}



	return 0;
}
