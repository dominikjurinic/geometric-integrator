#include <stdio.h>
#include <math.h>


void new_eul(struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double F_t[4][1], double T_t[3][1], double T_d[3][1], double omega[3][1], double omegam[4][1], double q[4][1], double dv[3][1], double domega[3][1]);




void new_eul(struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double F_t[4][1], double T_t[3][1], double T_d[3][1], double omega[3][1], double omegam[4][1], double q[4][1], double dv[3][1], double domega[3][1]){


//initialize function local omega
double localOmega[3][1] = {0};
localOmega[0][0] = omega[0][0];
localOmega[1][0] = omega[1][0];
localOmega[2][0] = omega[2][0];


double R[3][3] = {0};
R[0][0] = 2 * (pow(q[0][0], 2) + pow(q[1][0], 2) - 0.5);
R[0][1] = 2 * (q[1][0] * q[2][0] - q[0][0] * q[3][0]);
R[0][2] = 2 * (q[1][0] * q[3][0] + q[0][0] * q[2][0]);
R[1][0] = 2 * (q[1][0] * q[2][0] + q[0][0] * q[3][0]);
R[1][1] = 2 * (pow(q[0][0], 2) + pow(q[2][0], 2) - 0.5);
R[1][2] = 2 * (q[2][0] * q[3][0] - q[0][0] * q[1][0]);
R[2][0] = 2 * (q[1][0] * q[3][0] - q[0][0] * q[2][0]);
R[2][1] = 2 * (q[2][0] * q[3][0] + q[0][0] * q[1][0]);
R[2][2] = 2 * (pow(q[0][0], 2) + pow(q[3][0], 2) - 0.5);

double sumOfFt = 0;
for(int i = 0; i < 4; i++){
	for(int j = 0; j < 1; j++){
		sumOfFt = sumOfFt + F_t[i][j];
	}
}

double vector[3][1] = {0};
vector[2][0] = sumOfFt;
double sum = 0;
double RvVector[3][1] = {0};
for(int i = 0; i < 3; i++){			
	for(int j = 0; j < 1; j++){		
		for(int k = 0; k < 3; k++){	
			sum = sum + R[i][k]*vector[k][j];
			}
			RvVector[i][j] = sum;
			sum = 0;
		}
	}


double toAddVector[3][1] = {0};
toAddVector[2][0] = SpecificAircraft.mass*9.80665;


for(int i = 0; i < 3; i++){
	for(int j = 0; j < 1; j++){
		dv[i][j] = (1./SpecificAircraft.mass)*(RvVector[i][j] + toAddVector[i][j]);
	}
}

 
double skew_out[3][3] = {0};
skew(localOmega, skew_out);


double firstResultMatrix[3][3] = {0};

double sum2 = 0;
for(int i = 0; i < 3; i++){			
	for(int j = 0; j < 3; j++){		 
		for(int k = 0; k < 3; k++){	
			sum2 = sum2 + skew_out[i][k]*SpecificAircraft.J[k][j];
			}
			firstResultMatrix[i][j] = sum2;
			sum2 = 0;
		}
	}


double sum3 = 0;
double secondTermVector[3][1] = {0};
for(int i = 0; i < 3; i++){			
	for(int j = 0; j < 1; j++){		 
		for(int k = 0; k < 3; k++){	
			sum3 = sum3 + firstResultMatrix[i][k]*localOmega[k][j];
			}
			secondTermVector[i][j] = sum3;
			sum3 = 0;
		}
	}

double vectorPieceWise[4][1] = {0};
for(int i = 0; i < 4; i++){
	for(int j = 0; j < 1; j++){
		vectorPieceWise[i][j] = omegam[i][j] * SpecificMotor.rot_dir[i][j];
	}
}

double sumVectorPieceWise = 0;
for(int i = 0; i < 4; i++){
	for(int j = 0; j < 1; j++){
		sumVectorPieceWise = sumVectorPieceWise + vectorPieceWise[i][j];
	}
}


double lastVector[3][1] = {0};
lastVector[2][0] = SpecificMotor.inertia * sumVectorPieceWise;


double thirdTermVector[3][1] = {0};
double sum4 = 0;
for(int i = 0; i < 3; i++){			
	for(int j = 0; j < 1; j++){		 
		for(int k = 0; k < 3; k++){	
			sum4 = sum4 + skew_out[i][k]*lastVector[k][j];
			}
			thirdTermVector[i][j] = sum4;
			sum4 = 0;
		}
	}

double variable[3][1] = {0};
for(int i = 0; i < 3; i++){
	for(int j = 0; j < 1; j++){
		variable[i][j] = T_t[i][j] + T_d[i][j] - secondTermVector[i][j] - thirdTermVector[i][j];
	}
}


////solve system Ax = b - LU Decomposition
//A = AircraftJ - diag mattrix 
//b = variable -sum of upper operation


double L[3][3] = {0};
double U[3][3] = {0};
double B[3] = {variable[0][0], variable[1][0], variable[2][0]};
double X[3] = {0};   //-- in the end pu variables from x to domega
double Y[3] = {0};
int i,j,k;
int n = 3;

for(j=0; j<n; j++)
    {
        for(i=0; i<n; i++)
        {
            if(i<=j)
            {
                U[i][j]=SpecificAircraft.J[i][j];
                for(k=0; k<i-1; k++)
                    U[i][j]-=L[i][k]*U[k][j];
                if(i==j)
                    L[i][j]=1;
                else
                    L[i][j]=0;
            }
            else
            {
                L[i][j]=SpecificAircraft.J[i][j];
                for(k=0; k<=j-1; k++)
                    L[i][j]-=L[i][k]*U[k][j];
                L[i][j]/=U[j][j];
                U[i][j]=0;
            }
        }
    }

    for(i=0; i<n; i++)
    {
        Y[i]=B[i];
        for(j=0; j<i; j++)
        {
            Y[i]-=L[i][j]*Y[j];
        }
    }

    for(i=n-1; i>=0; i--)
    {
        X[i]= Y[i];
        for(j=i+1; j<n; j++)
        {
            X[i]-=U[i][j]*X[j];
        }
        X[i]/=U[i][i];
    }

domega[0][0] = X[0];	
domega[1][0] = X[1];
domega[2][0] = X[2];



}
