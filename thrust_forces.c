#include <stdio.h>
#include <math.h>

void thrust_forces(struct Motor SpecificMotor, double rpm[4][1], double F_t[4][1]); 	



 
void thrust_forces(struct Motor SpecificMotor, double rpm[4][1], double F_t[4][1]){


double localRpm[4][1] = {{pow(rpm[0][0], 2)},{pow(rpm[1][0], 2)}, {pow(rpm[2][0], 2)}, {pow(rpm[3][0], 2)}};

for(int i = 0; i < 4; i++){
	for(int j = 0; j < 1; j++){
		F_t[i][j] = - SpecificMotor.k_t * localRpm[i][j];
	}
}

}
