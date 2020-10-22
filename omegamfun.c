#include <stdio.h>
#include <string.h>
#include <math.h>

#define PI 3.14159265358979323846

void omegamfun(char type[], double omegam_out[4][1]);	


void omegamfun(char type[], double omegam_out[4][1]){


if(strcmp(type,"roll") == 0){
	omegam_out[0][0] = (2*PI)/(60) * (0 * sqrt(1/6.11E-8) * 1 + sqrt(1.4516625/6.11E-8) * 0);
	omegam_out[1][0] = (2*PI)/(60) * (0 * sqrt(1/6.11E-8) * 1 + sqrt(1.4516625/6.11E-8) * 0);
	omegam_out[2][0] = (2*PI)/(60) * (0 * sqrt(1/6.11E-8) * 0 + sqrt(1.4516625/6.11E-8) * 1);
	omegam_out[3][0] = (2*PI)/(60) * (0 * sqrt(1/6.11e-8) * 0 + sqrt(1.4516625/6.11E-8) * 1);	
}

if(strcmp(type,"yaw") == 0){
	omegam_out[0][0] = (2*PI)/(60) * (sqrt(1.2/6.11E-8) * 1 + sqrt(1.2516625/6.11E-8) * 0);
	omegam_out[1][0] = (2*PI)/(60) * (sqrt(1.2/6.11E-8) * 0 + sqrt(1.2516625/6.11E-8) * 1);
	omegam_out[2][0] = (2*PI)/(60) * (sqrt(1.2/6.11E-8) * 1 + sqrt(1.2516625/6.11E-8) * 0);
	omegam_out[3][0] = (2*PI)/(60) * (sqrt(1.2/6.11e-8) * 0 + sqrt(1.2516625/6.11E-8) * 1);	
}

if(strcmp(type,"yaw_climb") == 0){
	omegam_out[0][0] = (2*PI)/(60) * (sqrt(1.3/6.11E-8) * 1 + sqrt(1.3516625/6.11E-8) * 0);
	omegam_out[1][0] = (2*PI)/(60) * (sqrt(1.3/6.11E-8) * 0 + sqrt(1.3516625/6.11E-8) * 1);
	omegam_out[2][0] = (2*PI)/(60) * (sqrt(1.3/6.11E-8) * 1 + sqrt(1.3516625/6.11E-8) * 0);
	omegam_out[3][0] = (2*PI)/(60) * (sqrt(1.3/6.11e-8) * 0 + sqrt(1.3516625/6.11E-8) * 1);	
}


}
