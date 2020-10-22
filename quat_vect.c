#include <stdio.h>

void quat_vect(double q[4][1], double v[3][1], double q_out[4][1]);


void quat_vect(double q[4][1], double v[3][1], double q_out[4][1]){

	double L[3][4] = {0};	
	double tL[4][3] = {0};	

	L[0][0] = -q[1][0];
	L[0][1] =  q[0][0];
    	L[0][2] =  q[3][0];
    	L[0][3] = -q[2][0];
    	L[1][0] = -q[2][0];
    	L[1][1] = -q[3][0];
    	L[1][2] =  q[0][0];
    	L[1][3] =  q[1][0];
    	L[2][0] = -q[3][0];
    	L[2][1] =  q[2][0];
    	L[2][2] = -q[1][0];
    	L[2][3] =  q[0][0];



for(int i = 0; i < 4; i++){
	for(int j = 0; j < 3; j++){
		tL[i][j] = L[j][i];
	}
}


double sum = 0;


	for(int i = 0; i < 4; i++){			
		for(int j = 0; j < 1; j++){		 
			for(int k = 0; k < 3; k++){	
				sum = sum + tL[i][k]*v[k][j];
			}
			q_out[i][j] = sum;
			sum = 0;
		}
	}

}
