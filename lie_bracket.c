#include <stdio.h>

void lie_bracket(double skew_x[][3], double skew_y[][3], double skew_out[][3]);



void lie_bracket(double skew_x[][3], double skew_y[][3], double skew_out[][3]){

	
double sum = 0;
double skew_XY[3][3] = {0};
double skew_YX[3][3] = {0};

for(int i = 0; i < 3; i++){			
	for(int j = 0; j < 3; j++){		
		for(int k = 0; k < 3; k++){	
			sum = sum + skew_x[i][k]*skew_y[k][j];
			}
			skew_XY[i][j] = sum;
			sum = 0;
		}
	}

for(int i = 0; i < 3; i++){			
	for(int j = 0; j < 3; j++){		
		for(int k = 0; k < 3; k++){	
			sum = sum + skew_y[i][k]*skew_x[k][j];
			}
			skew_YX[i][j] = sum;
			sum = 0;
		}
	}

double difference = 0;
for(int i = 0; i < 3; i++){
	for(int j = 0; j < 3; j++){
		difference = skew_XY[i][j] - skew_YX[i][j];		
		skew_out[i][j] = difference;
		difference = 0;
	}
}



}
