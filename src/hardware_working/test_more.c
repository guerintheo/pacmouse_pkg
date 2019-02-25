#include "vl6180_pi/include/vl6180_pi.h"
#include <stdio.h>

int main(){

	vl6180 handle = vl6180_initialise(1);

	if(handle<=0){
		return 1;
	}
	while (1) {	
	set_scaling(handle,1);
	int distance1 = get_distance(handle);

	set_scaling(handle,2);
	int distance2 = get_distance(handle);
	
	set_scaling(handle,3);
	int distance3 = get_distance(handle);
	printf("%d\t%d\t%d\n", distance1, distance2, distance3);

	}

	return 0;
}
