
struct Motor {

	double k_d;
	double k_t; 
	double rot_dir[4][1];
	double inertia;
};


struct Aircraft {

	double mass;
	double J[3][3];
	double lev_arm;
	double matrix_FT[3][4];
};


