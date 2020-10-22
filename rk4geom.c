#include <stdio.h>
#include <string.h>
#include <math.h>

void rk4geom(double t, double h, struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double omega[3][1], double v[3][1], double q[4][1], double p[3][1], char type[], double omega_out[3][1], double v_out[3][1], double q_out[4][1], double p_out[3][1]);



void rk4geom(double t, double h, struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double omega[3][1], double v[3][1], double q[4][1], double p[3][1], char type[], double omega_out[3][1], double v_out[3][1], double q_out[4][1], double p_out[3][1]){

    double L[3][4] = {0};
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

    double appendedMatrix[4][4] = {0};
    appendedMatrix[0][0] = q[0][0];
    appendedMatrix[1][0] = q[1][0];
    appendedMatrix[2][0] = q[2][0];
    appendedMatrix[3][0] = q[3][0];

    appendedMatrix[0][1] = L[0][0];
    appendedMatrix[1][1] = L[0][1];
    appendedMatrix[2][1] = L[0][2];
    appendedMatrix[3][1] = L[0][3];

    appendedMatrix[0][2] = L[1][0];
    appendedMatrix[1][2] = L[1][1];
    appendedMatrix[2][2] = L[1][2];
    appendedMatrix[3][2] = L[1][3];

    appendedMatrix[0][3] = L[2][0];
    appendedMatrix[1][3] = L[2][1];
    appendedMatrix[2][3] = L[2][2];
    appendedMatrix[3][3] = L[2][3];

double omegam[4][1] = {0};
double rpm_out[4][1] = {0};
omegamfun(type, omegam);    
om2rpm(omegam, rpm_out); 

double F_t[4][1] = {0};
double T_t[3][1] = {0};
double T_d[3][1] = {0};
thrust_forces(SpecificMotor, rpm_out, F_t);  
thrust_torque(SpecificMotor, SpecificAircraft, rpm_out, T_t); 
drag_torque(SpecificMotor, rpm_out, T_d);

double vk1[3][1] = {0};
double k1[3][1] = {0};

new_eul(SpecificMotor, SpecificAircraft, F_t, T_t, T_d, omega, omegam, q, vk1, k1);

double u[3][1] = {0};
double uk1[3][1] = {0};
dexp(omega, u, uk1);

double pk1[3][1] = {{v[0][0]}, {v[1][0]}, {v[2][0]}};

omegamfun(type, omegam);  
double vk2[3][1] = {0};
double k2[3][1] = {0};

double newOmega[3][1] = {0};
newOmega[0][0] = omega[0][0] + 0.5 * h * k1[0][0];
newOmega[1][0] = omega[1][0] + 0.5 * h * k1[1][0];
newOmega[2][0] = omega[2][0] + 0.5 * h * k1[2][0];


double inputExp[3][1] = {{0.5 * h * uk1[0][0]},{0.5 * h * uk1[1][0]}, {0.5 * h * uk1[2][0]}};
double outputExp[4][1] = {0};
exp_S3(inputExp, outputExp);  

double newq[4][1] = {0}; 
for(int i = 0; i < 4; i++){			
	for(int j = 0; j < 1; j++){		
		for(int k = 0; k < 4; k++){	
			newq[i][j] = newq[i][j] + appendedMatrix[i][k]*outputExp[k][j];
			}
		}
	}

new_eul(SpecificMotor, SpecificAircraft, F_t, T_t, T_d, newOmega, omegam, newq, vk2, k2);


double omegaForDexp[3][1] = {0};
omegaForDexp[0][0] = omega[0][0] + 0.5 * h * k1[0][0];
omegaForDexp[1][0] = omega[1][0] + 0.5 * h * k1[1][0];
omegaForDexp[2][0] = omega[2][0] + 0.5 * h * k1[2][0];

double uForDexp[3][1] = {0};
uForDexp[0][0] = 0.5 * h * uk1[0][0];
uForDexp[1][0] = 0.5 * h * uk1[1][0];
uForDexp[2][0] = 0.5 * h * uk1[2][0];

double uk2[3][1] = {0};
dexp(omegaForDexp, uForDexp, uk2);

double pk2[3][1] = {0};
pk2[0][0] = v[0][0] + 0.5 * h * vk1[0][0];
pk2[1][0] = v[1][0] + 0.5 * h * vk1[1][0];
pk2[2][0] = v[2][0] + 0.5 * h * vk1[2][0];

omegamfun(type, omegam);   
double vk3[3][1] = {0};
double k3[3][1] = {0};
double newOmegaThird[3][1] = {0};
newOmegaThird[0][0] = omega[0][0] + 0.5 * h * k2[0][0];
newOmegaThird[1][0] = omega[1][0] + 0.5 * h * k2[1][0];
newOmegaThird[2][0] = omega[2][0] + 0.5 * h * k2[2][0];


double inputExpThird[3][1] = {{0.5 * h * uk2[0][0]},{0.5 * h * uk2[1][0]}, {0.5 * h * uk2[2][0]}};
double outputExpThird[4][1] = {0};
exp_S3(inputExpThird, outputExpThird);  

double newqThird[4][1] = {0};
for(int i = 0; i < 4; i++){			
	for(int j = 0; j < 1; j++){		 
		for(int k = 0; k < 4; k++){	
			newqThird[i][j] = newqThird[i][j] + appendedMatrix[i][k]*outputExpThird[k][j];
			}
		}
	}


new_eul(SpecificMotor, SpecificAircraft, F_t, T_t, T_d, newOmegaThird, omegam, newqThird, vk3, k3);


double omegaForDexpThird[3][1] = {0};
omegaForDexpThird[0][0] = omega[0][0] + 0.5 * h * k2[0][0];
omegaForDexpThird[1][0] = omega[1][0] + 0.5 * h * k2[1][0];
omegaForDexpThird[2][0] = omega[2][0] + 0.5 * h * k2[2][0];

double uForDexpThird[3][1] = {0};
uForDexpThird[0][0] = 0.5 * h * uk2[0][0];
uForDexpThird[1][0] = 0.5 * h * uk2[1][0];
uForDexpThird[2][0] = 0.5 * h * uk2[2][0];

double uk3[3][1] = {0};
dexp(omegaForDexpThird, uForDexpThird, uk3);

double pk3[3][1] = {0};
pk3[0][0] = v[0][0] + 0.5 * h * vk2[0][0];
pk3[1][0] = v[1][0] + 0.5 * h * vk2[1][0];
pk3[2][0] = v[2][0] + 0.5 * h * vk2[2][0];


omegamfun(type, omegam);  
double vk4[3][1] = {0};
double k4[3][1] = {0};
double newOmegaFourth[3][1] = {0};
newOmegaFourth[0][0] = omega[0][0] + h * k3[0][0];
newOmegaFourth[1][0] = omega[1][0] + h * k3[1][0];
newOmegaFourth[2][0] = omega[2][0] + h * k3[2][0];


double inputExpFourth[3][1] = {{h * uk3[0][0]},{h * uk3[1][0]}, {h * uk3[2][0]}};
double outputExpFourth[4][1] = {0};
exp_S3(inputExpFourth, outputExpFourth);  

double newqFourth[4][1] = {0}; 
for(int i = 0; i < 4; i++){			
	for(int j = 0; j < 1; j++){		 
		for(int k = 0; k < 4; k++){
			newqFourth[i][j] = newqFourth[i][j] + appendedMatrix[i][k]*outputExpFourth[k][j];
			}
		}
	}


new_eul(SpecificMotor, SpecificAircraft, F_t, T_t, T_d, newOmegaFourth, omegam, newqFourth, vk4, k4);

double omegaForDexpFourth[3][1] = {0};
omegaForDexpFourth[0][0] = omega[0][0] + h * k3[0][0];
omegaForDexpFourth[1][0] = omega[1][0] + h * k3[1][0];
omegaForDexpFourth[2][0] = omega[2][0] + h * k3[2][0];

double uForDexpFourth[3][1] = {0};
uForDexpFourth[0][0] = h * uk3[0][0];
uForDexpFourth[1][0] = h * uk3[1][0];
uForDexpFourth[2][0] = h * uk3[2][0];

double uk4[3][1] = {0};
dexp(omegaForDexpFourth, uForDexpFourth, uk4);

double pk4[3][1] = {0};
pk4[0][0] = v[0][0] + h * vk3[0][0];
pk4[1][0] = v[1][0] + h * vk3[1][0];
pk4[2][0] = v[2][0] + h * vk3[2][0];

omega_out[0][0] = omega[0][0] + 1./6. * h * (k1[0][0] + 2 * k2[0][0] + 2*k3[0][0] + k4[0][0]);
omega_out[1][0] = omega[1][0] + 1./6. * h * (k1[1][0] + 2 * k2[1][0] + 2*k3[1][0] + k4[1][0]);
omega_out[2][0] = omega[2][0] + 1./6. * h * (k1[2][0] + 2 * k2[2][0] + 2*k3[2][0] + k4[2][0]);

double inputExpEnd[3][1] = {0};
double outExpEnd[4][1] = {0};

inputExpEnd[0][0] = 1./6. * h * (uk1[0][0]+ 2 * uk2[0][0] + 2 * uk3[0][0] + uk4[0][0]);
inputExpEnd[1][0] = 1./6. * h * (uk1[1][0]+ 2 * uk2[1][0] + 2 * uk3[1][0] + uk4[1][0]);
inputExpEnd[2][0] = 1./6. * h * (uk1[2][0]+ 2 * uk2[2][0] + 2 * uk3[2][0] + uk4[2][0]);

exp_S3(inputExpEnd, outExpEnd);


for(int i = 0; i < 4; i++){			
	for(int j = 0; j < 1; j++){		 
		for(int k = 0; k < 4; k++){	
			q_out[i][j] = q_out[i][j] + appendedMatrix[i][k]*outExpEnd[k][j];
			}
		}
	}


v_out[0][0] = v[0][0] +  1./6. * h *(vk1[0][0] + 2 * vk2[0][0] + 2 * vk3[0][0] + vk4[0][0]);
v_out[1][0] = v[1][0] +  1./6. * h *(vk1[1][0] + 2 * vk2[1][0] + 2 * vk3[1][0] + vk4[1][0]);
v_out[2][0] = v[2][0] +  1./6. * h *(vk1[2][0] + 2 * vk2[2][0] + 2 * vk3[2][0] + vk4[2][0]);


p_out[0][0] = p[0][0] + 1./6. * h * (pk1[0][0] + 2 * pk2[0][0] + 2 * pk3[0][0] + pk4[0][0]);
p_out[1][0] = p[1][0] + 1./6. * h * (pk1[1][0] + 2 * pk2[1][0] + 2 * pk3[1][0] + pk4[1][0]);
p_out[2][0] = p[2][0] + 1./6. * h * (pk1[2][0] + 2 * pk2[2][0] + 2 * pk3[2][0] + pk4[2][0]);



}
