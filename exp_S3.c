#include <stdio.h>
#include <math.h>

void exp_S3(double u[3][1], double u_S3[4][1]);	


void exp_S3(double u[3][1], double u_S3[4][1]){

double norm_u = 0;
double unitQuaternion [4][1] = {{1},{0},{0},{0}};
double firstTransitionQuaternion[4][1] = {{1},{0},{0},{0}};	
double secondTransitionQuaternion [4][1] = {0};
secondTransitionQuaternion[1][0] = u[0][0];
secondTransitionQuaternion[2][0] = u[1][0];
secondTransitionQuaternion[3][0] = u[2][0];


norm_u = sqrt(pow(u[0][0],2)+pow(u[1][0],2)+pow(u[2][0],2));


for(int i = 0; i < 4; i++){
	for(int j = 0; j < 1; j++){
		firstTransitionQuaternion[i][j] = cosf(0.5*norm_u)*firstTransitionQuaternion[i]	   [j];		
	}
}


for(int i = 0; i < 4; i++){
	for(int j = 0; j < 1; j++){
		secondTransitionQuaternion[i][j] = (sinf(0.5*norm_u))/(norm_u) * secondTransitionQuaternion[i][j];
	}
}
if(norm_u < pow(10,-12)){
	double U_S3[4][1] = {{1},{0},{0},{0}};
}
else{	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 1; j++){
			u_S3[i][j] = firstTransitionQuaternion[i][j] + secondTransitionQuaternion[i][j];
		}
	}
	
}


}
