// Built in headers
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>

// Aircraft Specific Headers
#include "acdata.h"
#include "aero_data_tables.h"
#include "prop_data_tables.h"

// Custom headers
#include "atmos_dat.h"
#include "constants.h"
#include "position.h"
#include "socketUDP.h"

// Flight Gear Header
#include "net_fdm.hxx"

// Structure Declarations
struct X{
	double x;
	double y;
	double z;
	double u;
	double v;
	double w;
	double psi;
	double theta;
	double phi;
	double p;
	double q;
	double r;
	double vt;
	double alfa;
	double beta;
	double north;
	double east;
	double down;
};

struct U{
	double delH;
	double delA;
	double delR;
	double Flap;
	double SpdBr;
	double Th;
	int    MODE;
};

struct data{
	
	double cax_alfa_spbr;
	double cax_alfa_q;
	double cax_alfa_q_flaple;
	double cax_alfa_beta_flaple;
	double cax_alfa_beta_hrzt;
	double cax_alfa_beta_0hrzt;
	double cax;
	double cay_alfa_p;
	double cay_alfa_p_flaple;
	double cay_alfa_r;
	double cay_alfa_r_flaple;
	double cay_alfa_beta;
	double cay_alfa_beta_flaple;
	double cay_alfa_beta_ail;
	double cay_alfa_beta_ail_flaple;
	double cay_alfa_beta_rdr;
	double cay;
	double caz_alfa_spbr;
	double caz_alfa_q;
	double caz_alfa_q_flaple;
	double caz_alfa_beta_flaple;
	double caz_alfa_beta_hrzt;
	double caz_alfa_beta_hrzt0;
	double caz;
	double cal_alfa_p;
	double cal_alfa_p_flaple;
	double cal_alfa_r;
	double cal_alfa_r_flaple;
	double cal_alfa_beta;
	double cal_alfa_beta_flaple;
	double cal_alfa_beta_ail;
	double cal_alfa_beta_ail_flaple;
	double cal_alfa_beta_rdr;
	double cal_alfa_beta_hrzt;
	double cal_alfa_beta_hrzt0;
	double cal;
	double cam_alfa;
	double mu_camabh;
	double cam_alfa_q_flaple;
	double cam_alfa_q;
	double cam_alfa_spbr;
	double cam_alfa_hrzt;
	double cam_alfa_beta_flaple;
	double cam_alfa_beta_hrzt;
	double cam_alfa_beta_hrzt0;
	double cam;
	double can_alfa_beta;
	double can_alfa_r_flaple;
	double can_alfa_r;
	double can_alfa_p_flaple;
	double can_alfa_p;
	double can_alfa_beta_rdr;
	double can_alfa_beta_ail_flaple;
	double can_alfa_beta_ail;
	double can_alfa_beta_flaple;
	double can_alfa_beta_hrzt;
	double can_alfa_beta_hrzt0;
	double can;
	double fpx;
	double delta;
	double ixx_inv;
	double iyy_inv;
	double izz_inv;
	double ixy_inv;
	double iyz_inv;
	double izx_inv;
};

struct atmdata{
	double temp;
	double pres;
	double rho;
	double vs;
	double qbar;
};

// Declare Function Prototypes
void inceptorPositionCheck();
void initializeSimulation();
void doReset();
void resetInitialization();
void modeProcessing( int mode);
double*** initialize3(double a1[20][19], double a2[20][19],double a3[20][19]);
double*** initialize3Prop(double a1[8][8], double a2[8][8],double a3[8][8]);
double*** initialize5(double a1[20][19], double a2[20][19],double a3[20][19],double a4[20][19],double a5[20][19]);
void export2FG(X curX, X xDot, U curU, GEO curPos, double heading, int socket, char* hostip, unsigned short port, FGNetFDM* fdm);
data getForces(data forces, X state, U in, double*** a1, double*** a2, double*** a3, double*** a4, double*** a5);
atmdata getEnvars(double z, double vt);
inline X getEOM (X state, U in, data forces, double*** a1, double*** a2, double*** a3, double*** a4, double*** a5, double*** a6);
double getFPX(double z, double vt, double th, atmdata envars, double*** a1);
X RK4(X prev, U in, double dt, data forces,double*** a1, double*** a2, double*** a3, double*** a4, double*** a5, double*** a6);
double htond (double x);
float htonf (float x);
void print3d(int lenx, int leny, int lenz, double*** array);
U proccessInceptors(int socket, int port, U lastU);
inline double limit( const double input, const double lower, const double upper);
double valueNearLast( double last, double current, double tolerance);
inline bool withinEpsilonOf(const double query_value, const double target_value, const double epsilon = 1.0E-12);
void write_data(FILE *fp, X x, GEO pos, double t);
U deadband( U input, double deadband);


void write_data(FILE *fp, X x, U u, GEO pos, double t){
	fprintf(fp,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",t,x.alfa, x.beta, x.x,
			x.y, x.z, x.u, x.v, x.w, x.phi, x.theta, x.psi, x.p, x.q, x.r, pos.LAT, pos.LON, pos.ALT, x.north, x.east, x.down,
			u.delH, u.delA, u.delR, u.Flap, u.SpdBr, u.Th);
	fflush(fp);
}

// Initialization Functions 

double*** initialize3(double a1[20][19], double a2[20][19],double a3[20][19]){

	// Create 3-d matricies
	int dimX = 3; 
	int dimY = 20; 
	int dimZ = 19;

	double*** array;    // 3D array definition;
	// begin memory allocation
	array = new double**[dimX];
	for(int x = 0; x < dimX; ++x) {
		array[x] = new double*[dimY];
		for(int y = 0; y < dimY; ++y) {
			array[x][y] = new double[dimZ];
			for(int z = 0; z < dimZ; ++z) { // initialize the values to whatever you want the default to be
				if(x == 0){
					array[x][y][z] = a1[y][z];
				}
				else if(x==1){
					array[x][y][z] = a2[y][z];
				}
				else {
					array[x][y][z] = a3[y][z];
				}
			}
		}
	}
	return array;
}

double*** initialize3Prop(double a1[8][8], double a2[8][8],double a3[8][8]){

	// Create 3-d matricies
	int dimX = 3; 
	int dimY = 8; 
	int dimZ = 8;

	double*** array;    // 3D array definition;
	// begin memory allocation
	array = new double**[dimX];
	for(int x = 0; x < dimX; ++x) {
		array[x] = new double*[dimY];
		for(int y = 0; y < dimY; ++y) {
			array[x][y] = new double[dimZ];
			for(int z = 0; z < dimZ; ++z) { // initialize the values to whatever you want the default to be
				if(x == 0){
					array[x][y][z] = a1[y][z];
				}
				else if(x==1){
					array[x][y][z] = a2[y][z];
				}
				else {
					array[x][y][z] = a3[y][z];
				}
			}
		}
	}
	return array;
}

double*** initialize5(double a1[20][19], double a2[20][19],double a3[20][19],double a4[20][19],double a5[20][19]){

// Create 3-d matricies
	int dimX = 5; 
	int dimY = 20; 
	int dimZ = 19;

	double*** array;    // 3D array definition;
	// begin memory allocation
	array = new double**[dimX];
	for(int x = 0; x < dimX; ++x) {
		array[x] = new double*[dimY];
		for(int y = 0; y < dimY; ++y) {
			array[x][y] = new double[dimZ];
			for(int z = 0; z < dimZ; ++z) { // initialize the values to whatever you want the default to be
				if(x == 0){
					array[x][y][z] = a1[y][z];
				}
				else if(x==1){
					array[x][y][z] = a2[y][z];
				}
				else if(x==2){
					array[x][y][z] = a3[y][z];
				}
				else if (x==3){
					array[x][y][z] = a4[y][z];
				}
				else{
					array[x][y][z] = a5[y][z];
				}
			}
		}
	}
	return array;
}

// Aircraft Dynamics Functions

data getForces(data forces, X state, U in, double*** a1, double*** a2, double*** a3, double*** a4, double*** a5){

	// Make sure alfa & Beta are in degrees.
	double alfa = state.alfa * R2D;
	double beta = state.beta * R2D;

	// Adjust inputs to their weighted values
	double Flap  = in.Flap  * FLAPLIMIT;
	double SpdBr = in.SpdBr * SPDBRLIMIT;
	double delH  = in.delH  * ELEVLIMIT;
	double delA  = in.delA  * AILLIMIT;
	double delR  = in.delR  * RUDLIMIT;

	//$$$$$$$$$$$$$$$$$$$$$$   Compute CAX $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	// 1-D Look ups
	forces.cax_alfa_spbr			= lookup1d(alfa,20,alfa_aero_tab,cax_alfa_spbr_tab);
	forces.cax_alfa_q				= lookup1d(alfa,20,alfa_aero_tab,cax_alfa_q_tab);
	forces.cax_alfa_q_flaple		= lookup1d(alfa,14,alfa_aero_flaple_tab,cax_alfa_q_flaple_tab);
	//2-D Look ups
	forces.cax_alfa_beta_flaple		= lookup2d(alfa,beta,14,19,alfa_aero_flaple_tab,beta_aero_tab,cax_alfa_beta_flaple_tab);
	//3-D Look ups
	forces.cax_alfa_beta_hrzt		= lookup3d(alfa,beta,delH,20,19,5,alfa_aero_tab,beta_aero_tab, hrzt_aero_long_tab,a1);
	forces.cax_alfa_beta_0hrzt		= lookup3d(alfa,beta,0,20,19,5,alfa_aero_tab,beta_aero_tab, hrzt_aero_long_tab,a1);

	//Intermediate caluclations
	double del_cax_alfa_beta_hrzt = forces.cax_alfa_beta_flaple - forces.cax_alfa_beta_0hrzt;

	//Build CAX from components
	forces.cax = forces.cax_alfa_beta_hrzt + 
		(del_cax_alfa_beta_hrzt*(1.0-(Flap/25.0))) +
		(forces.cax_alfa_spbr*(SpdBr/60.0)) + ((forces.cax_alfa_q +
		(forces.cax_alfa_q_flaple*(1.0-(Flap/25.0))))*((cbar*state.q)/(2.0*state.vt)));

	//$$$$$$$$$$$$$$$$$$$$$$   Compute CAY $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    //1-D Look ups
	forces.cay_alfa_p					= lookup1d(alfa,20,alfa_aero_tab,cay_alfa_p_tab);
	forces.cay_alfa_p_flaple			= lookup1d(alfa,14,alfa_aero_flaple_tab,cay_alfa_p_flaple_tab);
	forces.cay_alfa_r					= lookup1d(alfa,20,alfa_aero_tab,cay_alfa_r_tab);
	forces.cay_alfa_r_flaple			= lookup1d(alfa,14,alfa_aero_flaple_tab, cay_alfa_r_flaple_tab);
	//2-D Look ups
	forces.cay_alfa_beta				= lookup2d(alfa,beta,20,19,alfa_aero_tab, beta_aero_tab, cay_alfa_beta_tab);
	forces.cay_alfa_beta_flaple			= lookup2d(alfa,beta,14,19,alfa_aero_flaple_tab, beta_aero_tab, cay_alfa_beta_flaple_tab);
	forces.cay_alfa_beta_ail			= lookup2d(alfa,beta,20,19,alfa_aero_tab,beta_aero_tab, cay_alfa_beta_ail_tab);
	forces.cay_alfa_beta_ail_flaple		= lookup2d(alfa,beta,14,19,alfa_aero_flaple_tab,beta_aero_tab, cay_alfa_beta_ail_flaple_tab);
	forces.cay_alfa_beta_rdr			= lookup2d(alfa,beta,20,19,alfa_aero_tab, beta_aero_tab, cay_alfa_beta_rdr_tab);

	//ntermediate Calculations
	double del_cay_alfa_beta_flaple     = forces.cay_alfa_beta_flaple - forces.cay_alfa_beta;
	double del_cay_alfa_beta_rdr        = forces.cay_alfa_beta_rdr - forces.cay_alfa_beta;
	double del_cay_alfa_beta_ail        = forces.cay_alfa_beta_ail - forces.cay_alfa_beta;
	double del_cay_alfa_beta_ail_flaple = forces.cay_alfa_beta_ail_flaple - forces.cay_alfa_beta_flaple - (forces.cay_alfa_beta_ail - forces.cay_alfa_beta);

	// Build CAY from components
	forces.cay = forces.cay_alfa_beta + 
		del_cay_alfa_beta_flaple*(1.0-Flap/25.0) +
		(del_cay_alfa_beta_ail + del_cay_alfa_beta_ail_flaple*(1.0-Flap/25.0))*(delA/20.0) +
		del_cay_alfa_beta_rdr*(delR/30.0)+ (forces.cay_alfa_p + forces.cay_alfa_p_flaple*(1.0-Flap/25.0))*(bbar*state.p)/(2.0*state.vt) +
		(forces.cay_alfa_r + forces.cay_alfa_r_flaple*(1.0-Flap/25.0))*(bbar*state.r)/(2.0*state.vt);

	//$$$$$$$$$$$$$$$$$$$$$$   Compute CAZ $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	//1-D Look ups
	forces.caz_alfa_spbr				= lookup1d(alfa,20,alfa_aero_tab,caz_alfa_spbr_tab);
	forces.caz_alfa_q					= lookup1d(alfa,20,alfa_aero_tab,caz_alfa_q_tab);
	forces.caz_alfa_q_flaple			= lookup1d(alfa,14,alfa_aero_flaple_tab, caz_alfa_q_flaple_tab);
	//2-D Look ups
	forces.caz_alfa_beta_flaple			= lookup2d(alfa,beta,14,19,alfa_aero_flaple_tab,beta_aero_tab,caz_alfa_beta_flaple_tab);
	//3-D Look ups
	forces.caz_alfa_beta_hrzt			= lookup3d(alfa,beta,delH,20,19,5,alfa_aero_tab,beta_aero_tab, hrzt_aero_long_tab,a2);
	forces.caz_alfa_beta_hrzt0		    = lookup3d(alfa,beta,0,20,19,5,alfa_aero_tab,beta_aero_tab, hrzt_aero_long_tab,a2);

	//Intermediate caluclations
	double del_caz_alfa_beta_flaple		= forces.caz_alfa_beta_flaple - forces.caz_alfa_beta_hrzt0;

	//Build CAZ from components  (Double Check)
	forces.caz = forces.caz_alfa_beta_hrzt + (del_caz_alfa_beta_flaple*(1.0-(Flap/25.0))) + (forces.caz_alfa_spbr*(SpdBr/60.0)) +
		((forces.caz_alfa_q + (forces.caz_alfa_q_flaple*(1.0-(Flap/25.0))))*((cbar*state.q)/(2.0*state.vt)));
	
	//$$$$$$$$$$$$$$$$$$$$$$   Compute CAL $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	// 1-D Look Ups
	 forces.cal_alfa_p					= lookup1d(alfa,20,alfa_aero_tab,cal_alfa_p_tab);
	 forces.cal_alfa_p_flaple			= lookup1d(alfa,14,alfa_aero_flaple_tab, cal_alfa_p_flaple_tab);
	 forces.cal_alfa_r					= lookup1d(alfa,20,alfa_aero_tab,cal_alfa_r_tab);
	 forces.cal_alfa_r_flaple			= lookup1d(alfa,14,alfa_aero_flaple_tab, cal_alfa_r_flaple_tab);
	 forces.cal_alfa_beta				= lookup1d(alfa,20,alfa_aero_tab, cal_alfa_beta_tab);
	 //2-D Look Ups
	 forces.cal_alfa_beta_flaple		= lookup2d(alfa,beta,14,19,alfa_aero_flaple_tab,beta_aero_tab,cal_alfa_beta_flaple_tab);
	 forces.cal_alfa_beta_ail			= lookup2d(alfa,beta,20,19,alfa_aero_tab,beta_aero_tab,cal_alfa_beta_ail_tab);
	 forces.cal_alfa_beta_ail_flaple    = lookup2d(alfa,beta,14,19,alfa_aero_flaple_tab,beta_aero_tab,cal_alfa_beta_ail_flaple_tab);
	 forces.cal_alfa_beta_rdr			= lookup2d(alfa,beta,20,19,alfa_aero_tab,beta_aero_tab,cal_alfa_beta_rdr_tab);
	 //3-D Look Ups
	 forces.cal_alfa_beta_hrzt			= lookup3d(alfa,beta,delH,20,19,3,alfa_aero_tab,beta_aero_tab,hrzt_aero_latdir_tab,a3);
	 forces.cal_alfa_beta_hrzt0			= lookup3d(alfa,beta,0,20,19,3,alfa_aero_tab,beta_aero_tab,hrzt_aero_latdir_tab,a3);
 
	 //Intermediate Calculations
	 double del_cal_alfa_beta_flaple	 = forces.cal_alfa_beta_flaple - forces.cal_alfa_beta_hrzt0;
	 double del_cal_alfa_beta_rdr	     = forces.cal_alfa_beta_rdr - forces.cal_alfa_beta_hrzt0;
	 double del_cal_alfa_beta_ail		 = forces.cal_alfa_beta_ail - forces.cal_alfa_beta_hrzt0;
	 double del_cal_alfa_beta_ail_flaple = forces.cal_alfa_beta_ail_flaple - forces.cal_alfa_beta_flaple - 
		 (forces.cal_alfa_beta_ail - forces.cal_alfa_beta_hrzt0);
 
	//Build CAL from components 
	forces.cal = forces.cal_alfa_beta_hrzt + del_cal_alfa_beta_flaple*(1.0-Flap/25.0) + forces.cal_alfa_beta*(D2R*beta)+
		(del_cal_alfa_beta_ail + del_cal_alfa_beta_ail_flaple*(1.0-Flap/25.0))*(delA/20.0) + del_cal_alfa_beta_rdr*(delR/30.0)+
		(forces.cal_alfa_p + forces.cal_alfa_p_flaple*(1.0-Flap/25.0))*(bbar*state.p)/(2.0*state.vt) +
		(forces.cal_alfa_r + forces.cal_alfa_r_flaple*(1.0-Flap/25.0))*(bbar*state.r)/(2.0*state.vt);
	
     //%$$$$$$$$$$$$$$$$$$$$$$   Compute CAM $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	 //1-D Look Ups
	 forces.cam_alfa					=  lookup1d(alfa,20,alfa_aero_tab,cam_alfa_tab);
	 forces.mu_camabh					= lookup1d(delH,5,hrzt_aero_long_tab,mu_camabh_tab);
	 forces.cam_alfa_q_flaple			= lookup1d(alfa,14,alfa_aero_flaple_tab,cam_alfa_q_flaple_tab);
	 forces.cam_alfa_q					= lookup1d(alfa,20,alfa_aero_tab,cam_alfa_q_tab);
	 forces.cam_alfa_spbr				= lookup1d(alfa,20,alfa_aero_tab,cam_alfa_spbr_tab);
	 //2-D Look Ups
	 forces.cam_alfa_hrzt				= lookup2d1(alfa,delH,20,7,alfa_aero_tab, hrzt_aero_long_cama_tab,cam_alfa_hrzt_tab);
	 forces.cam_alfa_beta_flaple		= lookup2d(alfa,beta,14,19,alfa_aero_flaple_tab,beta_aero_tab,cam_alfa_beta_flaple_tab);
	 //3-D Look Ups
	 forces.cam_alfa_beta_hrzt			= lookup3d(alfa,beta,delH,20,19,5,alfa_aero_tab,beta_aero_tab,hrzt_aero_long_tab ,a4);
	 forces.cam_alfa_beta_hrzt0			= lookup3d(alfa,beta,0,20,19,5,alfa_aero_tab,beta_aero_tab,hrzt_aero_long_tab , a4);
 
	 //Intermediate Calculations
	 double del_cam_alfa_beta_flaple	= forces.cam_alfa_beta_flaple - forces.cam_alfa_beta_hrzt0;

	 //Build CAM out of components
	 forces.cam = (forces.cam_alfa_beta_hrzt*forces.mu_camabh) + 
		 (del_cam_alfa_beta_flaple*(1.0-(Flap/25.0))) +
		 (forces.cam_alfa_spbr*(SpdBr/60.0)) +
		 ((forces.cam_alfa_q + forces.cam_alfa_q_flaple*(1.0-(Flap/25.0)))*
		 ((cbar*state.q)/(2.0*state.vt))) + 
		 forces.cam_alfa +
		 forces.cam_alfa_hrzt + 
		 (forces.caz*(xcmbr-xcmb));
	
	 //$$$$$$$$$$$$$$$$$$$$$$   Compute CAN $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	 //1-D Look Ups
	 //can_alfa_ail  = lookup1d(alfa,alfa_aero_tab,can_alfa_ail_tab);
	 forces.can_alfa_beta				= lookup1d(alfa,20,alfa_aero_tab,can_alfa_beta_tab);
	 forces.can_alfa_r_flaple			= lookup1d(alfa,14,alfa_aero_flaple_tab, can_alfa_r_flaple_tab);
	 forces.can_alfa_r					= lookup1d(alfa,20,alfa_aero_tab,can_alfa_r_tab);
	 forces.can_alfa_p_flaple			= lookup1d(alfa,14,alfa_aero_flaple_tab, can_alfa_p_flaple_tab);
	 forces.can_alfa_p					= lookup1d(alfa,20,alfa_aero_tab,can_alfa_p_tab);
	 //2-D Look Ups
	 forces.can_alfa_beta_rdr		    = lookup2d(alfa,beta,20,19,alfa_aero_tab,beta_aero_tab,can_alfa_beta_rdr_tab);
	 forces.can_alfa_beta_ail_flaple	= lookup2d(alfa,beta,14,19,alfa_aero_flaple_tab,beta_aero_tab,can_alfa_beta_ail_flaple_tab);
	 forces.can_alfa_beta_ail			= lookup2d(alfa,beta,20,19,alfa_aero_tab,beta_aero_tab,can_alfa_beta_ail_tab);
	 forces.can_alfa_beta_flaple		= lookup2d(alfa,beta, 14,19,alfa_aero_flaple_tab,beta_aero_tab,can_alfa_beta_flaple_tab);
	 //3-D Look Ups
	 forces.can_alfa_beta_hrzt			= lookup3d(alfa,beta,delH,20,19,3,alfa_aero_tab,beta_aero_tab,hrzt_aero_latdir_tab, a5);
	 forces.can_alfa_beta_hrzt0			= lookup3d(alfa,beta,0,20,19,3,alfa_aero_tab,beta_aero_tab,hrzt_aero_latdir_tab,a5);

	 //Intermediate Calculations
	double del_can_alfa_beta_flaple		= forces.can_alfa_beta_flaple - forces.can_alfa_beta_hrzt0;
	double del_can_alfa_beta_rdr		= forces.can_alfa_beta_rdr - forces.can_alfa_beta_hrzt0;
	double del_can_alfa_beta_ail		= forces.can_alfa_beta_ail - forces.can_alfa_beta_hrzt0;
	double del_can_alfa_beta_ail_flaple = forces.can_alfa_beta_ail_flaple - forces.can_alfa_beta_flaple - 
		(forces.can_alfa_beta_ail - forces.can_alfa_beta_hrzt0);

	//Build CAN out of components
	forces.can = forces.can_alfa_beta_hrzt + del_can_alfa_beta_flaple*(1.0-Flap/25.0) + forces.can_alfa_beta*(D2R*beta)
		+ (del_can_alfa_beta_ail + del_can_alfa_beta_ail_flaple*(1.0-Flap/25.0))*(delA/20.0) + del_can_alfa_beta_rdr*(delR/30.0)
		+ (forces.can_alfa_p + forces.can_alfa_p_flaple*(1.0-Flap/25.0))*(bbar*state.p)/(2.0*state.vt) +
		(forces.can_alfa_r + forces.can_alfa_r_flaple*(1.0-Flap/25.0))*(bbar*state.r)/(2.0*state.vt)
		- forces.cay*(xcmbr-xcmb)*cbar/bbar;

	return forces;
}


atmdata getEnvars(double z, double vt){

	atmdata envars;
	double h = -1.0 * z;

	envars.temp = lookup1d(h,102,alt_e_tab,temp_e_tab);
	envars.pres = lookup1d(h,102,alt_e_tab,pres_e_tab);
	envars.rho  = lookup1d(h,102,alt_e_tab,rho_e_tab);
	envars.vs   = lookup1d(h,102,alt_e_tab,vs_e_tab);

	envars.qbar = 0.5*envars.rho*pow(vt,2.0);

	return envars;
}

double getFPX(double z, double vt, double th, atmdata envars, double*** a1){

	double h = -1.0 * z;
	double mach = vt/envars.vs;
	double pwr;

	if(th < thtl_ab_on_off){
		pwr = k_thtl_ab_off_1*th + k_thtl_ab_off_0;
	}else if( th >= thtl_ab_on_off){
		pwr = k_thtl_ab_on_1*th + k_thtl_ab_on_0;
	}

	return lookup3d(h,mach,pwr,8,8,3,alt_prop_tab,mach_prop_tab,powr_prop_tab,
		a1);
}

inline X getEOM (X state, U in, data forces, double*** a1, double*** a2, double*** a3, double*** a4, double*** a5, double*** a6){
	// C++ math library trig functions require data in radians
	X dot;
	atmdata envars;

	// maybe just pass through state
	state.vt   = sqrt(pow(state.u,2.0)+pow(state.v,2.0)+pow(state.w,2.0));	    //  ft/s
	state.alfa = atan2(state.w,state.u);									    //  rad / Make sure alfa/beta are degrees for forces lookup.
	state.beta = atan2(state.v,sqrt(pow(state.u,2.0)+pow(state.w,2.0)));	    //  rad

	dot.vt   = state.vt;
	dot.alfa = state.alfa;
	dot.beta = state.beta;

	forces     = getForces(forces,state,in,a1,a2,a3,a4,a5);					// unitless
	envars     = getEnvars(state.z,state.vt);								// temp= Farenheit, Pres = lbf/in^2, rho= slug/ft^3, Vs = ft/s, qbar = unitless
	forces.fpx = getFPX(state.z,state.vt,in.Th,envars,a6);					// fpx = lbf
	
	// Build forces
	double fax = envars.qbar * sbar * forces.cax;
	double fay = envars.qbar * sbar * forces.cay;
	double faz = envars.qbar * sbar * forces.caz;
	double la  = envars.qbar * sbar * bbar * forces.cal;
	double ma  = envars.qbar * sbar * cbar * forces.cam;
	double na  = envars.qbar * sbar * bbar * forces.can;

	// calculate gravity  // Potential problem
	double gx = -1.0*sin(state.theta)*g;					// Theta = rad
	double gy =  cos(state.theta)*sin(state.phi)*g;			// Theta & phi = rad
	double gz =  cos(state.theta)*cos(state.phi)*g;			// Theta & phi = rad

	// Position
	dot.x = (cos(state.psi)*cos(state.theta)*state.u)+	    // Theta, phi, psi = rad
     ((cos(state.psi)*sin(state.theta)*sin(state.phi))-		// u,v,w = ft/s
      (sin(state.psi)*cos(state.phi)))*state.v + 
     ((cos(state.psi)*sin(state.theta)*cos(state.phi))+ 
      (sin(state.psi)*sin(state.phi)))*state.w;

	dot.y = (sin(state.psi)*cos(state.theta)*state.u)+		// Theta, phi, psi = rad
     ((sin(state.psi)*sin(state.theta)*sin(state.phi))+		// u,v,w = ft/s
	 (cos(state.psi)*cos(state.phi)))*state.v + 
     ((sin(state.psi)*sin(state.theta)*cos(state.phi))- 
	 (cos(state.psi)*sin(state.phi)))*state.w;

	dot.z = (-1.0*sin(state.theta)*state.u)+(cos(state.theta)*
		sin(state.phi)*state.v)+(cos(state.theta)*cos(state.phi)*state.w);


	// Velocity
	dot.u = state.v*state.r - state.w*state.q + gx + ( (1.0/mass) * (fax + forces.fpx));

	dot.v = state.w*state.p - state.u*state.r + gy + ( (1.0/mass) * (fay + fpy));

	dot.w = state.u*state.q - state.v*state.p + gz + ( (1.0/mass) * (faz + fpz));

	// Euler Angles
	dot.theta = cos(state.phi)*state.q - sin(state.phi)*state.r;

	dot.phi = state.p +((sin(state.theta)*sin(state.phi))/cos(state.theta))*state.q + 
		((sin(state.theta)*cos(state.phi))/cos(state.theta))*state.r;

	dot.psi = (sin(state.phi)/cos(state.theta))*state.q + (cos(state.phi)/cos(state.theta))*state.r;


	// Angular velocity
	double H_b_x = (iyy-izz)*state.q*state.r + iyz*(pow(state.q,2.0)-pow(state.r,2.0)) + (izx*state.q - ixy*state.r)*state.p + la + lp;
	double H_b_y = (izz-ixx)*state.r*state.p + izx*(pow(state.r,2.0)-pow(state.p,2.0)) + (ixy*state.r - iyz*state.p)*state.q + ma + mp;
	double H_b_z = (ixx-iyy)*state.p*state.q + ixy*(pow(state.p,2.0)-pow(state.q,2.0)) + (iyz*state.p - izx*state.q)*state.r + na + np;

	dot.p = forces.ixx_inv*H_b_x + forces.ixy_inv*H_b_y + forces.izx_inv*H_b_z;
	dot.q = forces.ixy_inv*H_b_x + forces.iyy_inv*H_b_y + forces.iyz_inv*H_b_z;
	dot.r = forces.izx_inv*H_b_x + forces.iyz_inv*H_b_y + forces.izz_inv*H_b_z;

	// North East Down
	dot.north = state.u*cos(state.theta)*sin(state.psi)
			+ state.v*((sin(state.phi)*sin(state.theta)*sin(state.psi)) + (cos(state.phi)*cos(state.psi)) )  // y-prime
			+ state.w*((cos(state.phi)*sin(state.theta)*sin(state.psi)) - (sin(state.phi)*cos(state.psi)));

	dot.east  = state.u*cos(state.theta)*cos(state.psi)
		    + state.v*((sin(state.phi)*sin(state.theta)*cos(state.psi)) - (cos(state.phi)*cos(state.psi)) )  // x-prime
		    + state.w*((cos(state.phi)*sin(state.theta)*cos(state.psi)) + (sin(state.phi)*sin(state.psi)));

	// z-prime
	dot.down  = -state.u*sin(state.theta) + state.v*sin(state.phi)*cos(state.theta) + state.w*cos(state.phi)*cos(state.theta);

	return dot;
}

X RK4(X prev, U in, double dt, data forces,double*** a1, double*** a2, double*** a3, double*** a4, double*** a5, double*** a6){

	double t = 0;
	X X[5], curX;

	X[0] = prev;

	for(int i = 0; i < 4; i++)
	{
		X[i+1] = getEOM(X[i],in,forces,a1,a2,a3,a4,a5,a6);

		if( i == 0 || i == 1)
		{
			t = dt/2.0;
		}
		else
		{
			t = dt;
		}

		X[i+1].x	 = X[0].x		+ t * X[i+1].x;
		X[i+1].y	 = X[0].y		+ t * X[i+1].y;
		X[i+1].z	 = X[0].z		+ t * X[i+1].z;
		X[i+1].u	 = X[0].u		+ t * X[i+1].u;
		X[i+1].v	 = X[0].v		+ t * X[i+1].v;
		X[i+1].w	 = X[0].w		+ t * X[i+1].w;
		X[i+1].phi   = X[0].phi		+ t * X[i+1].phi;
		X[i+1].theta = X[0].theta   + t * X[i+1].theta;
		X[i+1].psi   = X[0].psi		+ t * X[i+1].psi;
		X[i+1].p	 = X[0].p		+ t * X[i+1].p;
		X[i+1].q	 = X[0].q		+ t * X[i+1].q;
		X[i+1].r	 = X[0].r		+ t * X[i+1].r;
		X[i+1].north = X[0].north   + t * X[i+1].north;
		X[i+1].east	 = X[0].east	+ t * X[i+1].east;
		X[i+1].down	 = X[0].down	+ t * X[i+1].down;
	}



	curX.x		= -0.5*X[0].x	  + FRACTION*(2.0*X[1].x	   + 4.0*X[2].x		+ 2.0*X[3].x	   + X[4].x);
	curX.y		= -0.5*X[0].y	  + FRACTION*(2.0*X[1].y	   + 4.0*X[2].y		+ 2.0*X[3].y	   + X[4].y);
	curX.z		= -0.5*X[0].z	  + FRACTION*(2.0*X[1].z	   + 4.0*X[2].z		+ 2.0*X[3].z	   + X[4].z);
	curX.u		= -0.5*X[0].u	  + FRACTION*(2.0*X[1].u	   + 4.0*X[2].u		+ 2.0*X[3].u	   + X[4].u);
	curX.v		= -0.5*X[0].v	  + FRACTION*(2.0*X[1].v	   + 4.0*X[2].v		+ 2.0*X[3].v	   + X[4].v);
	curX.w		= -0.5*X[0].w	  + FRACTION*(2.0*X[1].w	   + 4.0*X[2].w	    + 2.0*X[3].w	   + X[4].w);
	curX.phi    = -0.5*X[0].phi   + FRACTION*(2.0*X[1].phi     + 4.0*X[2].phi   + 2.0*X[3].phi     + X[4].phi);
	curX.theta  = -0.5*X[0].theta + FRACTION*(2.0*X[1].theta   + 4.0*X[2].theta + 2.0*X[3].theta   + X[4].theta);
	curX.psi    = -0.5*X[0].psi   + FRACTION*(2.0*X[1].psi     + 4.0*X[2].psi   + 2.0*X[3].psi     + X[4].psi);
	curX.p		= -0.5*X[0].p	  + FRACTION*(2.0*X[1].p	   + 4.0*X[2].p		+ 2.0*X[3].p	   + X[4].p);
	curX.q		= -0.5*X[0].q	  + FRACTION*(2.0*X[1].q	   + 4.0*X[2].q		+ 2.0*X[3].q	   + X[4].q);
	curX.r		= -0.5*X[0].r	  + FRACTION*(2.0*X[1].r	   + 4.0*X[2].r		+ 2.0*X[3].r	   + X[4].r);
	curX.north	= -0.5*X[0].north + FRACTION*(2.0*X[1].north   + 4.0*X[2].north	+ 2.0*X[3].north   + X[4].north);
	curX.east	= -0.5*X[0].east  + FRACTION*(2.0*X[1].east	   + 4.0*X[2].east	+ 2.0*X[3].east	   + X[4].east);
	curX.down	= -0.5*X[0].down  + FRACTION*(2.0*X[1].down	   + 4.0*X[2].down	+ 2.0*X[3].down	   + X[4].down);

	curX.alfa = atan2(curX.w,curX.u);									    //  rad / Make sure alfa/beta are degrees for forces lookup.
	curX.beta = atan2(curX.v,sqrt(pow(curX.u,2.0)+pow(curX.w,2.0)));

	return curX;
}


// Flight Gear Handling functions
void export2FG(X curX, X xDot, U curU, GEO curPos, double heading, int socket, char* hostip, unsigned short port, FGNetFDM* fdm){

	// Position Info
	fdm->latitude  = htond(curPos.LAT * D2R);
	fdm->longitude = htond(curPos.LON * D2R);
	fdm->altitude  = htond(curPos.ALT*ft2m );  // fg altitude can be in meters or ft. Using feet.

	// Orientation
	fdm->phi	  = htonf(static_cast<float>(curX.phi)	);
	fdm->theta    = htonf(static_cast<float>(curX.theta));
	fdm->psi      = htonf(static_cast<float>(heading*D2R));
	fdm->alpha    =	htonf(static_cast<float>(curX.alfa));
	fdm->beta     = htonf(static_cast<float>(curX.beta));
	fdm->slip_deg = htonf(static_cast<float>(curX.beta*R2D));

	// Velocities
	fdm->climb_rate = htonf(static_cast<float>(curX.w)); // ft/s
	fdm->phidot     = htonf(static_cast<float>(xDot.phi));
	fdm->thetadot   = htonf(static_cast<float>(xDot.theta));
	fdm->psidot     = htonf(static_cast<float>(xDot.psi));
	fdm->v_north  	= htonf(static_cast<float>(xDot.north));
	fdm->v_east     = htonf(static_cast<float>(xDot.east));
	fdm->v_down		= htonf(static_cast<float>(xDot.down));
	/*
	  float vcas;	// calibrated airspeed
	*/
	fdm->vcas = htonf(static_cast<float>(curX.vt*fps2knots));


	// Misc Info
	fdm->num_engines = htonl(1);
	fdm->num_tanks = htonl(1);
	fdm->fuel_quantity[0] = htonf(100.0);
	fdm->num_wheels = htonl(3);

	//fdm.cur_time = htonl(time(0));
	fdm->warp = htonl(1);

	// Control Surface Positions
	fdm->elevator   = htonf(static_cast<float>( curU.delH/ELEVLIMIT ));
	fdm->rudder     = htonf(static_cast<float>( curU.delR/RUDLIMIT ));
	fdm->speedbrake = htonf(static_cast<float>( curU.SpdBr/SPDBRLIMIT));

	fdm->left_flap  = htonf(static_cast<float>( curU.Flap/FLAPLIMIT));
	fdm->right_flap = htonf(static_cast<float>( curU.Flap/FLAPLIMIT));

	fdm->left_aileron  = htonf(static_cast<float>( -1.0*curU.delA/AILLIMIT));
	fdm->right_aileron = htonf(static_cast<float>( -1.0*curU.delA/AILLIMIT));

	socketUDP_write(socket, hostip, port, reinterpret_cast<char*>(fdm), sizeof(*fdm));

}

U proccessInceptors(int socket, int port, U lastU)
{
	U ctrls;
	//int nBytes = 0;
	int pos = -1;
	char buffer[115];
	int nBytes = 115;
	double val[12];
	bool foundNum = false;
	char * ptr;
    const double lower_lim = -1.0;
    const double mid_lim   = 0.0;
    const double upper_lim = 1.0;

	while( recvfrom(socket, buffer, nBytes, 0, NULL, NULL) > 0)
	{
		//std::cout << "Buff size is: " << sizeof(buffer) << std::endl;
		pos = -1;

		for(int i = 0; i < nBytes; i++)
		{
			if( buffer[i] == '#' && !foundNum)
			{
				foundNum = true;
				pos++;
			}

			if( foundNum )
			{
				ptr = &buffer[i+1];

				if( buffer[i+1] == '-')
				{
					char temp[9];
					memcpy(temp,ptr,9);
					val[pos] = atof(temp);
				}
				else
				{
					char temp[8];
					memcpy(temp,ptr,8);
					val[pos] = atof(temp);
				}
				foundNum = false;
			}

		}
	}

	// The next lines of code essentially grooms the input. The dynamics
	// model is intolerant of band input values, so it is imperative to
	// make sure what goes into it is accurate.


	// Store values, limit them to their hard limits.
	// Hardware allows possibility of values going over +/- 1. So
	// check that here.

	val[1] += val[10];
	val[2] += val[9];
	val[0] += -1 * val[11];



	ctrls.delH  = limit( -1.0*val[0],       lower_lim, upper_lim );
	ctrls.delA  = limit( val[1], 	  		lower_lim, upper_lim );
	ctrls.delR  = limit( val[2], 			lower_lim, upper_lim);
	ctrls.Th    = limit( (val[3]+val[4])/2, mid_lim,   0.99 );    // Using 0.99 for thtl as special case
	ctrls.Flap  = limit( val[6], 			mid_lim,   upper_lim);
	ctrls.SpdBr = limit( val[7], 			mid_lim,   upper_lim);

	// Make sure that the new value is not wildly away from the last value.
	// This is a sort of edge detection.
	const double tolerance = 0.2;


	ctrls.delH  = valueNearLast( lastU.delH , ctrls.delH , tolerance);
	ctrls.delA  = valueNearLast( lastU.delA , ctrls.delA , tolerance);
	ctrls.delR  = valueNearLast( lastU.delR , ctrls.delR , 0.15);
	ctrls.Th    = valueNearLast( lastU.Th   , ctrls.Th   , tolerance);
	ctrls.Flap  = valueNearLast( lastU.Flap , ctrls.Flap , 0.1);
	ctrls.SpdBr = valueNearLast( lastU.SpdBr, ctrls.SpdBr, tolerance);


	// Apply a deadband so really small values don't add noise.
	ctrls = deadband(ctrls, 0.05);

	ctrls.MODE = val[8];

	/*
	std::cout << "################################" << std::endl;
	std::cout << "Elevator pos: " << ctrls.delH  << std::endl;
	std::cout << "Aileron  pos: " << ctrls.delA << std::endl;
	std::cout << "Rudder   pos: " << ctrls.delR << std::endl;
	std::cout << "Throttle pos: " << ctrls.Th   << std::endl;
	std::cout << "Flap     pos: " << ctrls.Flap << std::endl;
	std::cout << "Speed Br pos: " << ctrls.SpdBr << std::endl;
	std::cout << "################################" << std::endl;
	*/


	return ctrls;
}

// This function acts as an edge detector. It checks to see if the current inceptor value
// is within a reasonable distance away from the last. This prevents "blips" in the input.
double valueNearLast( double last, double current, double tolerance)
{
	if( !withinEpsilonOf( current, last, tolerance))
	{
		return last;
	}
	else
	{
		return current;
	}
}

// This function makes the value of any control zero if it is within the dead band of zero
U deadband( U input, double deadband)
{
	U ctrls;

	const double thtl_deadband = 0.02;
	const double rud_deadband  = 0.1;
	const double elev_deadband = 0.01;

	ctrls = input;

	if( withinEpsilonOf( input.delH, 0, elev_deadband))
	{
		ctrls.delH = 0;
	}

	if( withinEpsilonOf( input.delA, 0, deadband))
	{
		ctrls.delA = 0;
	}

	if( withinEpsilonOf( input.delR, 0, rud_deadband))
	{
		ctrls.delR = 0;
	}

	if( withinEpsilonOf( input.Th, 0, thtl_deadband))
	{
		ctrls.Th = 0;
	}

	if( withinEpsilonOf( input.Flap, 0, deadband))
	{
		ctrls.Flap = 0;
	}

	if( withinEpsilonOf( input.SpdBr, 0, deadband))
	{
		ctrls.SpdBr = 0;
	}

	return ctrls;
}

inline double limit( const double input, const double lower, const double upper)
{
	return std::min(std::max(input,lower),upper);
}

inline bool withinEpsilonOf(const double query_value,
                            const double target_value,
                            const double epsilon)
{
  return (query_value >= (target_value - epsilon)) &&
    (query_value <= (target_value + epsilon));
}


double htond (double x)	
{
	int * p = (int*)&x;
	int tmp = p[0];
	p[0] = htonl(p[1]);
	p[1] = htonl(tmp);

	return x;
}

float htonf (float x) // returns x as a float
{
	int * p = (int *)&x;
	*p = htonl(*p);
	return x;
}

// TEST & DEBUG ROUTINES
void print3d (int lenx, int leny, int lenz, double*** array){

	for(int i=0;i<lenz;i++){
		for(int j=0;j<lenx;j++){
			for(int k=0;k<leny;k++){
				printf("%f ",array[i][j][k]);
			}
			printf("\n");
		}
		printf("\n");
	}
}

