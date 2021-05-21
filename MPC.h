double* MPC(double** xr, double x[3], int Nu, int itmax, double Q[3], double R[2], float to ,int sample, double r, double L);
double**  cercleRef(int max_iter, double to, double freq, double R, double Xo, double Yo, double theta0);
double**  rightRef(int max_iter, double to, double a);
