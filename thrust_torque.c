#include <stdio.h>
#include <math.h> 
 
void thrust_torque(struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double rpm[4][1], double T_t[4][1]);


void thrust_torque(struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double rpm[4][1], double T_t[4][1]){

double F_t[4][1] = {0};
thrust_forces(SpecificMotor, rpm, F_t); 


double sum = 0;
for(int i = 0; i < 3; i++){			
	for(int j = 0; j < 1; j++){		 
		for(int k = 0; k < 4; k++){	
			sum = sum + SpecificAircraft.matrix_FT[i][k]*F_t[k][j];
			}
			T_t[i][j] = sum * SpecificAircraft.lev_arm;
			sum = 0;
		}
	}

}


