#include <stdio.h>
#include <math.h>


void drag_torque(struct Motor SpecificMotor, double rpm[4][1], double T_d[3][1]){


T_d[0][0] = 0;
T_d[1][0] = 0;
double sum = 0;
double totalSum = 0;

for(int i = 0; i < 4; i++){
	for(int j = 0; j < 1; j++){
		sum = SpecificMotor.k_d * rpm[i][j] * fabs(rpm[i][j]) * SpecificMotor.rot_dir[i][j];
	}
	totalSum = totalSum + sum;
	sum = 0;	
}
T_d[2][0] = totalSum;

}




