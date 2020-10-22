#include <stdio.h>

void skew(double x[3][1], double x_skew[3][3]);	


void skew(double x[3][1], double x_skew[3][3]){	

x_skew[0][0] = 0;	
x_skew[1][0] = x[0][2];
x_skew[2][0] = (-1) * x[1][0];
x_skew[0][1] = (-1) * x[2][0];
x_skew[1][1] = 0;
x_skew[2][1] = x[0][0];
x_skew[0][2] = x[1][0];
x_skew[1][2] = (-1) * x[0][0];
x_skew[2][2] = 0;

}
