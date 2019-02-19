#include "vl6180_pi/include/vl6180_pi.h"
#include <stdio.h>
#include <wiringPi.h>
#include <unistd.h>

// cc Wall tof_test.c -o tof_test -lvl6180_pi  -lwiringPi

int main(){
	setbuf(stdout, NULL);
	int power_pins[] = {22, 23, 24, 25, 26, 27};
	char addresses[] = {0x26, 0x27, 0x28, 0x29, 0x30, 0x31};
	vl6180 handles[6];

	wiringPiSetup();
	
	for (int i=0; i < 6; i++) {
		pinMode(power_pins[i], OUTPUT);
		digitalWrite(power_pins[i], LOW);
	}
	printf("VL6180s LOW\n");
	sleep(1);

	for (int i=0; i < 6; i++) {
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
			set_scaling(handles[i], 1);
		}
	}

	int distances[6];
	while (1) {  
		for (int i=0; i < 6; i++) {
			distances[i] = get_distance(handles[i]);
		}
		printf("%d\t%d\t%d\t%d\t%d\t%d\n", distances[0], distances[1], distances[2], distances[3], distances[4], distances[5]);
	}

	return 0;
}
