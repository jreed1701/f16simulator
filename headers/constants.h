/*
 * constants.h
 *
 *  Created on: Jun 8, 2013
 *      Author: dev
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define MODE_EXIT       4
#define MODE_RESET      3
#define MODE_HOLD       2
#define MODE_OPERATE    1
#define EXIT_SUCCESS    0

#define g 32.174 //%ft/s^2
#define nm2ft (5280.0/1.0)*1.1516 //%ft/nm
// Conversions
#define m2ft 3.28084  // meters to feet
#define ft2m 0.3048   // feet to meters
#define fps2knots 0.59248

#define PI 3.141592653589793238462

#define D2R ( PI / 180.0)
#define R2D (180.0 / PI )

#define SEC2MICROSEC 1000000

#define FRACTION (1.0/6.0)


#endif /* CONSTANTS_H_ */
