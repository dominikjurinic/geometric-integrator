#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846

void om2rpm(double omega[4][1], double rpm_out[4][1]); //input is quternion [4x1]





void om2rpm(double omega[4][1], double rpm_out[4][1]){
	
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 1; j++){
			rpm_out[i][j] = 60 *(omega[i][j])/(2*PI);
		}
	}
}

