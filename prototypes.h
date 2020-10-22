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




void dexp(double omega[3][1], double u[3][1], double du[3][1]);

void drag_torque(struct Motor SpecificMotor, double rpm[4][1], double T_d[3][1]);

void exp_S3(double u[3][1], double u_S3[4][1]);

void lie_bracket(double skew_x[][3], double skew_y[][3], double skew_out[][3]);

void new_eul(struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double F_t[4][1], double T_t[3][1], double T_d[3][1], double omega[3][1], double omegam[4][1], double q[4][1], double dv[3][1], double domega[3][1]);

void om2rpm(double omega[4][1], double rpm_out[4][1]);

void omegamfun(char type[], double omegam_out[4][1]);

void quat_rot(double q[4][1], double v[3][1], double v_out[3][1]);

void quat_vect(double q[4][1], double v[3][1], double q_out[4][1]);

void skew(double x[3][1], double x_skew[3][3]);

void SO3_to_R3(double x_skew[3][3], double x[3][1]);

void thrust_forces(struct Motor SpecificMotor, double rpm[4][1], double F_t[4][1]);

void thrust_torque(struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double rpm[4][1], double T_t[4][1]);


void rk2cl(double t, double h, struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double omega[3][1], double v[3][1], double q[4][1], double p[3][1], char type[], double omega_out[3][1], double v_out[3][1], double q_out[4][1], double p_out[3][1]);


void rk2geom(double t, double h, struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double omega[3][1], double v[3][1], double q[4][1], double p[3][1], char type[], double omega_out[3][1], double v_out[3][1], double q_out[4][1], double p_out[3][1]);

void rk4cl(double t, double h, struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double omega[3][1], double v[3][1], double q[4][1], double p[3][1], char type[], double omega_out[3][1], double v_out[3][1], double q_out[4][1], double p_out[3][1]);


void rk4geom(double t, double h, struct Motor SpecificMotor, struct Aircraft SpecificAircraft, double omega[3][1], double v[3][1], double q[4][1], double p[3][1], char type[], double omega_out[3][1], double v_out[3][1], double q_out[4][1], double p_out[3][1]);



