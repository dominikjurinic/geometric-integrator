#include <stdio.h>
#include <string.h>
#include <math.h>


void rk4cl(double t, double h, struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double omega[3][1], double v[3][1], double q[4][1], double p[3][1], char type[], double omega_out[3][1], double v_out[3][1], double q_out[4][1], double p_out[3][1]);



void rk4cl(double t, double h, struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double omega[3][1], double v[3][1], double q[4][1], double p[3][1], char type[], double omega_out[3][1], double v_out[3][1], double q_out[4][1], double p_out[3][1]){

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


double qk1[4][1] = {0};
quat_vect(q,omega,qk1);

for(int i = 0; i < 4; i++){
	for(int j = 0; j < 1; j++){
		qk1[i][j] = 0.5 * qk1[i][j];
	}
}
double pk1[3][1] = {{v[0][0]},{v[1][0]},{v[2][0]}};


omegamfun(type, omegam);  
double vk2[3][1] = {0};
double k2[3][1] = {0};

double newOmega[3][1] = {{omega[0][0] + 0.5 * h * k1[0][0]},{omega[1][0] + 0.5 * h * k1[1][0]},{omega[2][0] + 0.5 * h * k1[2][0]}};


double newq[4][1] = {{q[0][0] + 0.5 * h * qk1[0][0]}, {q[1][0] + 0.5 * h * qk1[1][0]}, {q[2][0] + 0.5 * h * qk1[2][0]}, {q[3][0] + 0.5 * h * qk1[3][0]}};


new_eul(SpecificMotor, SpecificAircraft, F_t, T_t, T_d, newOmega, omegam, newq, vk2, k2);

double qk2[4][1] = {0};
quat_vect(newq, newOmega, qk2);

for(int i = 0; i < 4; i++){
	for(int j = 0; j < 1; j++){
		qk2[i][j] = 0.5 * qk2[i][j];
	}
}
double pk2[3][1] = {{v[0][0] + 0.5 * h * vk1[0][0]}, {v[1][0] + 0.5 * h * vk1[1][0]}, {v[2][0] + 0.5 * h * vk1[2][0]}};


omegamfun(type, omegam);
double vk3[3][1] = {0};
double k3[3][1] = {0};

double newOmegaThree[3][1] = {{omega[0][0] + 0.5 * h * k2[0][0]},{omega[1][0] + 0.5 * h * k2[1][0]},{omega[2][0] + 0.5 * h * k2[2][0]}};

double newqThree[4][1] = {{q[0][0] + 0.5 * h * qk2[0][0]}, {q[1][0] + 0.5 * h * qk2[1][0]}, {q[2][0] + 0.5 * h * qk2[2][0]}, {q[3][0] + 0.5 * h * qk2[3][0]}};

new_eul(SpecificMotor, SpecificAircraft, F_t, T_t, T_d, newOmegaThree, omegam, newqThree, vk3, k3);

double qk3[4][1] = {0};
quat_vect(newqThree, newOmegaThree, qk3);

for(int i = 0; i < 4; i++){
	for(int j = 0; j < 1; j++){
		qk3[i][j] = 0.5 * qk3[i][j];
	}
}

double pk3[3][1] = {{v[0][0] + 0.5 * h * vk2[0][0]}, {v[1][0] + 0.5 * h * vk2[1][0]}, {v[2][0] + 0.5 * h * vk2[2][0]}};



double vk4[3][1] = {0};
double k4[3][1] = {0};

double newOmegaFour[3][1] = {{omega[0][0] + h * k3[0][0]},{omega[1][0] + h * k3[1][0]},{omega[2][0] + h * k3[2][0]}};

double newqFour[4][1] = {{q[0][0] + h * qk3[0][0]}, {q[1][0] + h * qk3[1][0]}, {q[2][0] + h * qk3[2][0]}, {q[3][0] + h * qk3[3][0]}};

new_eul(SpecificMotor, SpecificAircraft, F_t, T_t, T_d, newOmegaFour, omegam, newqFour, vk4, k4);

double qk4[4][1] = {0};
quat_vect(newqFour, newOmegaFour, qk4);

for(int i = 0; i < 4; i++){
	for(int j = 0; j < 1; j++){
		qk4[i][j] = 0.5 * qk4[i][j];
	}
}

double pk4[3][1] = {{v[0][0] + h * vk3[0][0]}, {v[1][0] + h * vk3[1][0]}, {v[2][0] + h * vk3[2][0]}};



omega_out[0][0] = omega[0][0] + 1./6. * h * (k1[0][0] + 2 * k2[0][0] + 2*k3[0][0] + k4[0][0]);
omega_out[1][0] = omega[1][0] + 1./6. * h * (k1[1][0] + 2 * k2[1][0] + 2*k3[1][0] + k4[1][0]);
omega_out[2][0] = omega[2][0] + 1./6. * h * (k1[2][0] + 2 * k2[2][0] + 2*k3[2][0] + k4[2][0]);


q_out[0][0] = q[0][0] + 1./6. * h * (qk1[0][0] + 2 * qk2[0][0] + 2 * qk3[0][0] + qk4[0][0]);
q_out[1][0] = q[1][0] + 1./6. * h * (qk1[1][0] + 2 * qk2[1][0] + 2 * qk3[1][0] + qk4[1][0]);
q_out[2][0] = q[2][0] + 1./6. * h * (qk1[2][0] + 2 * qk2[2][0] + 2 * qk3[2][0] + qk4[2][0]);
q_out[3][0] = q[3][0] + 1./6. * h * (qk1[3][0] + 2 * qk2[3][0] + 2 * qk3[3][0] + qk4[3][0]);


double q_outTransposed[1][4] = {q_out[0][0],q_out[1][0],q_out[2][0], q_out[3][0]};
double sumToBeSquared = 0;
 
for(int i = 0; i < 1; i++){			
	for(int j = 0; j < 1; j++){		
		for(int k = 0; k < 4; k++){	
			sumToBeSquared = sumToBeSquared + q_outTransposed[i][k]*q_out[k][j];
			}
		}
	}

q_out[0][0] = q_out[0][0]/sqrt(sumToBeSquared);
q_out[1][0] = q_out[1][0]/sqrt(sumToBeSquared);
q_out[2][0] = q_out[2][0]/sqrt(sumToBeSquared);
q_out[3][0] = q_out[3][0]/sqrt(sumToBeSquared);



v_out[0][0] = v[0][0] +  1./6. * h *(vk1[0][0] + 2 * vk2[0][0] + 2 * vk3[0][0] + vk4[0][0]);
v_out[1][0] = v[1][0] +  1./6. * h *(vk1[1][0] + 2 * vk2[1][0] + 2 * vk3[1][0] + vk4[1][0]);
v_out[2][0] = v[2][0] +  1./6. * h *(vk1[2][0] + 2 * vk2[2][0] + 2 * vk3[2][0] + vk4[2][0]);


p_out[0][0] = p[0][0] + 1./6. * h * (pk1[0][0] + 2 * pk2[0][0] + 2 * pk3[0][0] + pk4[0][0]);
p_out[1][0] = p[1][0] + 1./6. * h * (pk1[1][0] + 2 * pk2[1][0] + 2 * pk3[1][0] + pk4[1][0]);
p_out[2][0] = p[2][0] + 1./6. * h * (pk1[2][0] + 2 * pk2[2][0] + 2 * pk3[2][0] + pk4[2][0]);



}
