#include <stdio.h>
#include <math.h>
#include <string.h>

#include "prototypes.h"

#include "omegamfun.c"
#include "thrust_forces.c"
#include "om2rpm.c"
#include "thrust_torque.c"
#include "drag_torque.c"
#include "new_eul.c"
#include "skew.c"
#include "quat_vect.c"
#include "SO3_to_R3.c"
#include "exp_S3.c"
#include "dexp.c"
#include "lie_bracket.c"

#include "rk2cl.c"
#include "rk2geom.c"
#include "rk4cl.c"
#include "rk4geom.c"


int main(){


struct Motor SpecificMotor; 
SpecificMotor.k_d = 1.5e-9;
SpecificMotor.k_t = 6.11e-8;
SpecificMotor.rot_dir[0][0] = 1;
SpecificMotor.rot_dir[1][0] = -1;
SpecificMotor.rot_dir[2][0] = 1;
SpecificMotor.rot_dir[3][0] = -1;
SpecificMotor.inertia = 0.5*0.011* pow(((8*2.54)*0.01),2);


struct Aircraft SpecificAircraft;
SpecificAircraft.mass = 0.5;
SpecificAircraft.J[0][0] = 0.00365;
SpecificAircraft.J[1][0] = 0.;
SpecificAircraft.J[2][0] = 0.;
SpecificAircraft.J[0][1] = 0.;
SpecificAircraft.J[1][1] = 0.00368;
SpecificAircraft.J[2][1] = 0.;
SpecificAircraft.J[0][2] = 0.;
SpecificAircraft.J[1][2] = 0.;
SpecificAircraft.J[2][2] = 0.00703;
SpecificAircraft.lev_arm = 0.17;
SpecificAircraft.matrix_FT[0][0] = 0.5 * sqrt(2);
SpecificAircraft.matrix_FT[0][1] = 0.5 * sqrt(2);
SpecificAircraft.matrix_FT[0][2] = -0.5 * sqrt(2);
SpecificAircraft.matrix_FT[0][3] = -0.5 * sqrt(2);
SpecificAircraft.matrix_FT[1][0] = -0.5 * sqrt(2);
SpecificAircraft.matrix_FT[1][1] = 0.5 * sqrt(2);
SpecificAircraft.matrix_FT[1][2] = 0.5 * sqrt(2);
SpecificAircraft.matrix_FT[1][3] = -0.5 * sqrt(2);
SpecificAircraft.matrix_FT[2][0] = 0;
SpecificAircraft.matrix_FT[2][1] = 0;
SpecificAircraft.matrix_FT[2][2] = 0;
SpecificAircraft.matrix_FT[2][3] = 0;



int N = 1000;
double h = 1./N;	//TIMESTEP

//printf("stepsize %lf", stepsize);  
double t[N];
double increment = 0;
for(int i = 0; i < N; i++){
    t[i] = increment;
    increment = increment + h;
}

double omegaVector[3][1000] = {0};
double velocityVector[3][1000] = {0};
double quaternion[4][1000] = {0};
quaternion[0][0] = 1;
double positionVector[3][1000] = {0};
positionVector[2][0] = -20;



double outOmega[3][1] = {0};
double outVelocity[3][1] = {0};
double outQuaternion[4][1] = {0};
double outPosition[3][1] = {0};





for(long int i = 1; i < N; i++){
double transitionOmega[3][1] = {{omegaVector[0][i-1]},{omegaVector[1][i-1]},{omegaVector[2][i-1]}};
double transitionQuaternion[4][1] = {{quaternion[0][i-1]},{quaternion[1][i-1]},{quaternion[2][i-1]},{quaternion[3][i-1]}};


double transitionVelocity[3][1] = {{velocityVector[0][i-1]},{velocityVector[1][i-1]},{velocityVector[2][i-1]}};
double transitionPosition[3][1] = {{positionVector[0][i-1]},{positionVector[1][i-1]},{positionVector[2][i-1]}};


double outOmega[3][1] = {0};
double outVelocity[3][1] = {0};
double outQuaternion[4][1] = {0};
double outPosition[3][1] = {0};


rk2geom(t[i], h, SpecificMotor, SpecificAircraft, transitionOmega, transitionVelocity, transitionQuaternion, transitionPosition, "roll", outOmega, outVelocity, outQuaternion, outPosition);



omegaVector[0][i] = outOmega[0][0];
omegaVector[1][i] = outOmega[1][0];
omegaVector[2][i] = outOmega[2][0];

velocityVector[0][i] = outVelocity[0][0];
velocityVector[1][i] = outVelocity[1][0];
velocityVector[2][i] = outVelocity[2][0];

quaternion[0][i] = outQuaternion[0][0];
quaternion[1][i] = outQuaternion[1][0];
quaternion[2][i] = outQuaternion[2][0];
quaternion[3][i] = outQuaternion[3][0];



positionVector[0][i] = outPosition[0][0];
positionVector[1][i] = outPosition[1][0];
positionVector[2][i] = outPosition[2][0];


}


return 0;
}
