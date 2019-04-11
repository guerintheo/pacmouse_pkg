#include <vl6180_pi/vl6180_pi.h>
#include <stdio.h>
#include <wiringPi.h>
#include <unistd.h>

// gcc tof_test.c -o tof_test -lvl6180_pi -lwiringPi

int main(){
	setbuf(stdout, NULL);
	int num_tofs = 5;
	
	int power_pins[] = {7, 19, 11, 23, 29}; // 6 and 7 are 31 and 37
	// removed lidar 4,5 on pin 23,29 at address 0x44,0x45
	char addresses[] = {0x40, 0x41, 0x42, 0x43, 0x44};
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
		sleep(0.3);
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
