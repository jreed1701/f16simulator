/*
 * position.h
 *
 *  Created on: Aug 8, 2013
 *      Author: production
 */

#ifndef POSITION_H_
#define POSITION_H_

#include "constants.h"
#include "math.h"

#define EQUATORIALRADIUS 6378137.0

struct GEO{
	double LAT;
	double LON;
	double ALT;
};

struct NED{
	double N;
	double E;
	double D;
};

GEO moveAircraftPosition(double range, double chi, GEO refPos);
double calcRange(double X, double Y);


// Input: range (meters), Heading (degrees), refPos is initialposition
GEO moveAircraftPosition(double range, double chi, GEO refPos)
{
	double LAT0 = refPos.LAT * D2R;
	double LON0 = refPos.LON * D2R;
	double hdg  = chi        * D2R;

	GEO newPos;

	// Pass through altitude
	newPos.ALT = refPos.ALT;

	double arc = range/EQUATORIALRADIUS;

	double newlat = asin( sin(LAT0)*cos(arc) + cos(LAT0)*sin(arc)*cos(hdg));

	double Y = sin(hdg)*sin(arc)*cos(LAT0);
	double X = cos(arc)-(sin(LAT0)*sin(newlat));

	double newlon = atan2(Y,X) + LON0;

	newPos.LAT = newlat * R2D;
	newPos.LON = newlon * R2D;

	return newPos;
}

double calcRange(double X, double Y)
{
	double xmeter = X*ft2m;
	double ymeter = Y*ft2m;
	return sqrt( pow(xmeter,2.0) + pow(ymeter,2.0) );
}


#endif /* POSITION_H_ */
