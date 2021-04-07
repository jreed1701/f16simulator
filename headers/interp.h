#ifndef INTERP_H
#define INTERP_H

#include <stdio.h>
#define LIMIT 0

double lookup1d(double x, int lenx, double xtab[], double ytab[]);
double lookup2d(double x1, double x2, int lenx1, int lenx2, double x1tab[], double x2tab[], double ytab[][19]);
double lookup2d0(double x1, double x2, int lenx1, int lenx2, double x1tab[], double x2tab[], double ytab[][8]);
double lookup2d1(double x1, double x2, int lenx1, int lenx2, double x1tab[], double x2tab[], double ytab[][7]);
double lookup3d(double x1, double x2, double x3, int lenx1, int lenx2, int lenx3, 
	double x1tab[], double x2tab[], double x3tab[], double*** array);

inline double lookup1d(double x, int lenx, double xtab[], double ytab[]){
	
	double y, delx, dely;

	// Compute location on X axis
	int j = 0;
    for (j=0; j < lenx; j++){
        if((x - xtab[j]) < 0){
            break;
        }
    }
    if(j > LIMIT){
        j -= 1;
    }
	// Compute Denominator of Slope DY/DX
    delx = xtab[j+1] - xtab[j];

    // Interpolate or Extrapolate Along X
    dely = ytab[j+1]-ytab[j];
    y    = ytab[j]+((x-xtab[j])*(dely/delx));

	return y;
}

inline double lookup2d(double x1, double x2, int lenx1, int lenx2, double x1tab[], double x2tab[], double ytab[][19]){

	//int lenx1 = sizeof(x1tab);
	//int lenx2 = sizeof(x2tab);
	double delx1, delx2, dely, dely1, dely2, y, y1, y2;

	// Find location on x1 axis
	int j1 = 0;
    for (j1=0; j1 < lenx1; j1++){
        if((x1 - x1tab[j1]) < 0){
            break;
        }
    }
    if(j1 > LIMIT){
        j1 -= 1;
    }

	// Find location on x2 axis
	int j2 = 0;
    for (j2=0; j2 < lenx2; j2++){
        if((x2 - x2tab[j2]) < 0){
            break;
        }
    }
    if(j2 > LIMIT){
        j2 -= 1;
    }

	// Compute Denominator Of Slopes DY/DX1 And DY/DX2
	delx1 = x1tab[j1+1] - x1tab[j1];
	delx2 = x2tab[j2+1] - x2tab[j2];

	// Interpolate Or Extrapolate Along Edges Parallel To X1
	dely1 = ytab[j1+1][j2]   - ytab[j1][j2];
	dely2 = ytab[j1+1][j2+1] - ytab[j1][j2+1];
	y1    = ytab[j1][j2]     + (x1 - x1tab[j1])*(dely1/delx1);
	y2    = ytab[j1][j2+1]   + (x1 - x1tab[j1])*(dely2/delx1);

	// Interpolate Or Extrapolate Across Interior Parallel To X2
	dely = y2 - y1;
	y    = y1 + (x2 - x2tab[j2])*(dely/delx2);

	return y;
}

double lookup2d0(double x1, double x2, int lenx1, int lenx2, double x1tab[], double x2tab[], double ytab[][8]){

	//int lenx1 = sizeof(x1tab);
	//int lenx2 = sizeof(x2tab);
	double delx1, delx2, dely, dely1, dely2, y, y1, y2;

	// Find location on x1 axis
	int j1 = 0;
    for (j1=0; j1 < lenx1; j1++){
        if((x1 - x1tab[j1]) < 0){
            break;
        }
    }
    if(j1 > LIMIT){
        j1 -= 1;
    }

	// Find location on x2 axis
	int j2 = 0;
    for (j2=0; j2 < lenx2; j2++){
        if((x2 - x2tab[j2]) < 0){
            break;
        }
    }
    if(j2 > LIMIT){
        j2 -= 1;
    }

	// Compute Denominator Of Slopes DY/DX1 And DY/DX2
	delx1 = x1tab[j1+1] - x1tab[j1];
	delx2 = x2tab[j2+1] - x2tab[j2];

	// Interpolate Or Extrapolate Along Edges Parallel To X1
	dely1 = ytab[j1+1][j2]   - ytab[j1][j2];
	dely2 = ytab[j1+1][j2+1] - ytab[j1][j2+1];
	y1    = ytab[j1][j2]     + (x1 - x1tab[j1])*(dely1/delx1);
	y2    = ytab[j1][j2+1]   + (x1 - x1tab[j1])*(dely2/delx1);

	// Interpolate Or Extrapolate Across Interior Parallel To X2
	dely = y2 - y1;
	y    = y1 + (x2 - x2tab[j2])*(dely/delx2);

	return y;
}

inline double lookup2d1(double x1, double x2, int lenx1, int lenx2, double x1tab[], double x2tab[], double ytab[][7]){

	//int lenx1 = sizeof(x1tab);
	//int lenx2 = sizeof(x2tab);
	double delx1, delx2, dely, dely1, dely2, y, y1, y2;

	// Find location on x1 axis
	int j1 = 0;
    for (j1=0; j1 < lenx1; j1++){
        if((x1 - x1tab[j1]) < 0){
            break;
        }
    }
    if(j1 > LIMIT){
        j1 -= 1;
    }

	// Find location on x2 axis
	int j2 = 0;
    for (j2=0; j2 < lenx2; j2++){
        if((x2 - x2tab[j2]) < 0){
            break;
        }
    }
    if(j2 > LIMIT){
        j2 -= 1;
    }

	// Compute Denominator Of Slopes DY/DX1 And DY/DX2
	delx1 = x1tab[j1+1] - x1tab[j1];
	delx2 = x2tab[j2+1] - x2tab[j2];

	// Interpolate Or Extrapolate Along Edges Parallel To X1
	dely1 = ytab[j1+1][j2]   - ytab[j1][j2];
	dely2 = ytab[j1+1][j2+1] - ytab[j1][j2+1];
	y1    = ytab[j1][j2]     + (x1 - x1tab[j1])*(dely1/delx1);
	y2    = ytab[j1][j2+1]   + (x1 - x1tab[j1])*(dely2/delx1);

	// Interpolate Or Extrapolate Across Interior Parallel To X2
	dely = y2 - y1;
	y    = y1 + (x2 - x2tab[j2])*(dely/delx2);

	return y;
}

double lookup3d(double x1, double x2, double x3, int lenx1, int lenx2, int lenx3, double x1tab[], double x2tab[], double x3tab[], double*** array){

	double delx1, delx2, delx3;
	double dely1_1, dely2_1, y1_1, y2_1;
	double dely1_2, dely2_2, y1_2, y2_2;
	double dely1, dely2, dely, y1, y2, y;

	// Find location on x1 axis
	int j1 = 0;
    for (j1=0; j1 < lenx1; j1++){
        if((x1 - x1tab[j1]) < 0){
            break;
        }
    }
    if(j1 > LIMIT){
        j1 -= 1;
    }

	// Find location on x2 axis
	int j2 = 0;
    for (j2=0; j2 < lenx2; j2++){
        if((x2 - x2tab[j2]) < 0){
            break;
        }
    }
    if(j2 > LIMIT){
        j2 -= 1;
    }

	// Find location on x3 axis
	int j3 = 0;
    for (j3=0; j3 < lenx3; j3++){
        if((x3 - x3tab[j3]) < 0){
            break;
        }
    }
    if(j3 > LIMIT){
        j3 -= 1;
    }

	// Compute Denominator Of Slopes DY/DX1, DY/DX2 And DY/DX3
	  delx1 = x1tab[j1+1] - x1tab[j1];
	  delx2 = x2tab[j2+1] - x2tab[j2];
	  delx3 = x3tab[j3+1] - x3tab[j3];

	  // 3d array c++ array[z][x][y]  matlab array[x][y][z]
	// Interpolate Or Extrapolate Along Edges Parallel To X1
	  dely1_1 = array[j3][j1+1][j2]   - array[j3][j1][j2];
	  dely2_1 = array[j3][j1+1][j2+1]   - array[j3][j1][j2+1];
	  y1_1    = array[j3][j1][j2]   + (x1 - x1tab[j1])*(dely1_1/delx1);
	  y2_1    = array[j3][j1 ][j2+1]   + (x1 - x1tab[j1])*(dely2_1/delx1);
 
	  dely1_2 = array[j3+1][j1+1][j2] - array[j3+1][j1][j2];
	  dely2_2 = array[j3+1][j1+1][j2+1]- array[j3+1][j1][j2+1];
	  y1_2    = array[j3+1][j1][j2] + (x1 - x1tab[j1])*(dely1_2/delx1);
	  y2_2    = array[j3+1][j1][j2+1] + (x1 - x1tab[j1])*(dely2_2/delx1);

	// Interpolate Or Extrapolate Across Interior Parallel To X2
	  dely1 = y2_1 - y1_1;
	  dely2 = y2_2 - y1_2;
	  y1    = y1_1 + (x2 - x2tab[j2])*(dely1/delx2);
	  y2    = y1_2 + (x2 - x2tab[j2])*(dely2/delx2);

	// Interpolate Or Extrapolate Across Interior Parallel To X3
	  dely = y2 - y1;
	  y    = y1 + (x3 - x3tab[j3])*(dely/delx3);

	return y;
}

#endif
