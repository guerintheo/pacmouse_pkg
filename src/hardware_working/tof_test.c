#include "vl6180_pi/include/vl6180_pi.h"
#include <stdio.h>
#include <wiringPi.h>
#include <unistd.h>

// gcc test.c -o test -L. -lvl6180_pi -lwiringPi -Wl,-rpath /home/pi/vl6180_pi

int main(){
	setbuf(stdout, NULL);
	int num_tofs = 5;
	int power_pins[] = {21, 22,23,24,25};
	char addresses[] = {0x40, 0x41, 0x42, 0x43, 0x44};
	vl6180 handles[num_tofs];

	wiringPiSetup();
	
	for (int i=0; i < num_tofs; i++) {
		pinMode(power_pins[i], OUTPUT);
		digitalWrite(power_pins[i], LOW);
	}
	printf("VL6180s LOW\n");
	sleep(1);

	for (int i=0; i < num_tofs; i++) {
		digitalWrite(power_pins[i], HIGH);
		sleep(1);
		handles[i] = vl6180_initialise(1);
		vl6180_change_addr(handles[i], addresses[i]);
		if (handles[i]<=0) {
			printf("Failed %x\n", addresses[i]);
			return 1;
		}
		else {
			printf("Init %x\n", addresses[i]);
			set_scaling(handles[i],1);
		}
	}

	int distances[num_tofs];
	while (1) {  
		for (int i=0; i < num_tofs; i++) {
			distances[i] = get_distance(handles[i]);
		}
		printf("%d\t%d\t%d\t%d\t%d\n", distances[0], distances[1], distances[2], distances[3], distances[4]);
	}



	return 0;
}
