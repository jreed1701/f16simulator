//#include <math.h>

#define sbar 300.0
#define cbar 11.32
#define bbar 30.0

#define mass (20500.0/32.174)
#define ixx 9496.0
#define iyy 55814.0
#define izz 63100.0
#define ixy 0.0
#define iyz 0.0
#define izx 982.0

#define xcmbr 0.35
#define xcmb  0.30

#define fpy 0
#define fpz 0
#define lp  0
#define mp  0
#define np  0

// Control input travel limits
#define FLAPLIMIT   25
#define SPDBRLIMIT  60
#define AILLIMIT    21.5
#define RUDLIMIT    30
#define ELEVLIMIT   25

//#define delta   (ixx*iyy*izz)-(ixx*pow(iyz,2.0))-(iyy*izx^2)-(izz*pow(ixy,2.0))-(2.0*ixy*iyz*izx);
//#define ixx_inv (iyy*izz-pow(iyz,2.0))/delta  
//#define ixy_inv (izz*ixy+izx*iyz)/delta //% neg
//#define iyy_inv (ixx*izz-pow(izx,2.0))/delta  
//#define iyz_inv (ixx*iyz+ixy*izx)/delta //% neg
//#define izz_inv (ixx*iyy-pow(ixy,2.0))/delta  
//#define izx_inv (iyy*izx+ixy*iyz)/delta //% neg
