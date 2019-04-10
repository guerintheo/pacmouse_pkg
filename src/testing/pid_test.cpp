#include "../estimation_control/pid.cpp"

int main(int argc, char **argv) {
	PID pid = PID(1.,1.,1.);

	float sp = 0;
	const float dt = 0.1;
	for (int i=0; i<100; i++) {
		sp = (i%10)/10.;
		printf("Set point: %f\t, value: %d\t, command: %f\n", sp, 0, pid.step(sp, dt));
	} 
}