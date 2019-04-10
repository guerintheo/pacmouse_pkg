#include <stdio.h>

class PID {
	float int_err, prev_err;

public:
	float kp, ki, kd;

	PID(float p, float i, float d) {
		kp = p;
		ki = i;
		kd = d;
	}

	float step(float err, float dt);
	void reset();
};

float PID::step(float err, float dt) {
	int_err += err * dt;

	const float d_err = (err - prev_err)/dt;
	prev_err = err;

	const float p = kp * err;
	const float i = ki * int_err;
	const float d = kd * d_err;

	return p + i + d;
}

void PID::reset() {
	int_err = 0;
	prev_err = 0;
}
