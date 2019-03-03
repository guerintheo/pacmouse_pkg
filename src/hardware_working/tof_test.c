#include "vl6180_pi/include/vl6180_pi.h"
#include <stdio.h>
#include <wiringPi.h>
#include <unistd.h>

// gcc test.c -o test -L. -lvl6180_pi -lwiringPi -Wl,-rpath /home/pi/vl6180_pi
// gcc tof_test.c -o tof_test -L. -lvl6180_pi -lwiringPi -Wl,-rpath /home/pi/pacmouse_pkg/src/hardware_working/vl6180_pi
int main(){
	setbuf(stdout, NULL);
	int num_tofs = 6;
	
	int power_pins[] = {7, 11, 19, 21, 23, 29}; // 6 and 7 are 31 and 37
	char addresses[] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45};
	vl6180 handles[num_tofs];

	wiringPiSetupPhys();
	
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
			printf("%d\t", distances[i]);
		}
		printf("\n");
	}



	return 0;
}
