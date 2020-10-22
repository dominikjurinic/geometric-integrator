#include <stdio.h>
#include <math.h>


void quat_rot(double q[4][1], double v[3][1], double v_out[3][1]);	


void quat_rot(double q[4][1], double v[3][1], double v_out[3][1]){

double R[3][3] = {0};
double Rmultiplied[3][3] = {0};
R[0][0] = powf(q[0][0], 2) + powf(q[1][0], 2) - 0.5 ;
R[0][1] = q[1][0] * q[2][0] - q[0][0] * q[3][0];
R[0][2] = q[1][0] * q[3][0] + q[0][0] * q[2][0];
R[1][0] = q[1][0] * q[2][0] + q[0][0] * q[3][0];
R[1][1] = powf(q[0][0], 2) + powf(q[2][0], 2) - 0.5;
R[1][2] = q[2][0] * q[3][0] - q[0][0] * q[1][0];
R[2][0] = q[1][0] * q[3][0] - q[0][0] * q[2][0];
R[2][1] = q[2][0] * q[3][0] + q[0][0] * q[1][0];
R[2][2] = powf(q[0][0], 2) + powf(q[3][0], 2) - 0.5;


double sum = 0;
for(int i = 0; i < 3; i++){			
	for(int j = 0; j < 1; j++){		 
		for(int k = 0; k < 3; k++){	
			sum = sum + 2*R[i][k]*v[k][j];
			}
			v_out[i][j] = sum;
			sum = 0;
		}
	}


}

