#include <stdio.h>
#include <math.h>

void dexp(double omega[3][1], double u[3][1], double du[3][1]);	    //protoype



void dexp(double omega[3][1], double u[3][1], double du[3][1]){

double norm_u = 0;
double skew_u[3][3] = {0};
double SO3_matrix[3][3] = {0};
norm_u = sqrt(pow(u[0][0],2)+pow(u[1][0],2)+pow(u[2][0],2));


skew(u, skew_u);		
skew(omega, SO3_matrix);	
SO3_to_R3(SO3_matrix, du);	

}

