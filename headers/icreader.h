/*
 * icreader.h
 *
 *  Created on: Sep 15, 2013
 *      Author: production
 */

#ifndef ICREADER_H_
#define ICREADER_H_

#include <string.h>
#include <iostream>
#include <fstream>

using std::ifstream;
using std::ostringstream;
using std::string;

bool proccessIcConfig(const std::string& file_name);
bool initializeStateConfiguration(std::ifstream& file, std::string& next_value);
bool initializeInceptorConfiguration(std::ifstream& file, std::string& next_value);
bool initializePosition(std::ifstream& file, std::string& next_value);
bool getNextToken(ifstream& file, string& next_token);
inline string& makeStringUppercase(string& s);

void parseCommandLine(int argc, char** argv)
{
  // Look through each argument and test for project specific arguments
  // only.
  for(int i = 0; i < argc; i++)
  {
    std::string testval = argv[i];
    // Check for initial conditions file
    if( !testval.compare("-icfile") )
    {
      const string filename = argv[i+1];
      proccessIcConfig(filename);
    }
    // Check if user specified a simulation rate
    else if( !testval.compare("-hertz"))
    {
    	HERTZ_COMMAND = static_cast<double>(atof(argv[i+1]));
    }
  }
}

//=============================================================================
// Process the IC File
//=============================================================================
bool proccessIcConfig(const std::string& file_name)
{

  ifstream file(file_name.c_str());

  // Check to see if file exists.
  if(!file)
  {
    std::cout << "Application error. IC File could not be opened. Exiting."
    		<< std::endl;
    return -1;
  }

  string next_value;

  getNextToken(file, next_value);

  // Go through file and look for MAJOR tags. Add an else-if and corresponding function
  // should new data be added to the IC file.
  while( file )
  {
    if( next_value[0] == '#')
    {
      char buffer[1024];
      file.getline(buffer,sizeof(buffer), '\n');
    }

    if( !next_value.compare("STATEVALUES") )
    {
      if( !initializeStateConfiguration(file, next_value) )
      {
        return false;
      }
    }
    else if( !next_value.compare("INCEPTORVALUES") )
    {
      if( !initializeInceptorConfiguration(file, next_value) )
      {
        return false;
      }
    }
    else if( !next_value.compare("POSITION") )
    {
      if( !initializePosition(file, next_value) )
      {
        return false;
      }
    }
    else
    {
      std::cout << "Unexpected Token. Check IC File. Exiting." << std::endl;
      return -1;
    }
  }
  return true;
}

//=============================================================================
// Get and Store Default Values for Aircraft trim state
//=============================================================================
bool initializeStateConfiguration(std::ifstream& file, std::string& next_value)
{

  string parameter;
  float value = 0.0;

  getNextToken( file, next_value );

  // Go through file and find MINOR tags and store the data next to it in its
  // respective variable.
  while( file )
  {
    if ( next_value[0] == '#' )
    {
      char buffer[1024];
      file.getline(buffer, sizeof(buffer), '\n');
    }

    parameter = next_value;
    getNextToken(file, next_value);
    value = static_cast<float>(atof(next_value.c_str()));

    if( !parameter.compare("ALPHA") )
    {
    	X0.alfa = value;
    }
    else if( !parameter.compare("BETA") )
    {
    	X0.beta = value;
    }
    else if( !parameter.compare("VT") )
    {
    	X0.vt = value;
    }
    else if( !parameter.compare("PHI") )
    {
    	X0.phi = value;
    }
    else if( !parameter.compare("THETA") )
    {
    	X0.theta = value;
    }
    else if( !parameter.compare("PSI") )
    {
    	X0.psi = value;
    }
    else
    {
      std::cout << "Did not find a parameter in State Configuration. Exiting!"
    		  << std::endl;
      return false;
    }
    getNextToken(file, next_value);
    if( !next_value.compare("INCEPTORVALUES"))
    {
    	return true;
    }
  }
  return true;
}

//=============================================================================
// Get and Store Default Values for Aircraft inceptor U0 trim state
//=============================================================================
bool initializeInceptorConfiguration(std::ifstream& file, std::string& next_value)
{

  string parameter;
  float value = 0.0;

  getNextToken( file, next_value );

  // Go through file and find MINOR tags and store the data next to it in its
  // respective variable.
  while( file )
  {
    if ( next_value[0] == '#' )
    {
      char buffer[1024];
      file.getline(buffer, sizeof(buffer), '\n');
    }

    parameter = next_value;
    getNextToken(file, next_value);
    value = static_cast<float>(atof(next_value.c_str()));

    if( !parameter.compare("DELH") )
    {
    	U0.delH = value;
    }
    else if( !parameter.compare("DELA") )
    {
    	U0.delA = value;
    }
    else if( !parameter.compare("DELR") )
    {
    	U0.delR = value;
    }
    else if( !parameter.compare("THTL") )
    {
    	U0.Th = value;
    }
    else if( !parameter.compare("SPDBR") )
    {
    	U0.SpdBr = value;
    }
    else if( !parameter.compare("FLAP") )
    {
    	U0.Flap = value;
    }
    else
    {
      std::cout << "Did not find a parameter in inceptor Configuration. Exiting!"
    		  << std::endl;
      return false;
    }
    getNextToken(file, next_value);
    if( !next_value.compare("POSITION"))
    {
    	return true;
    }
  }
  return true;
}

//=============================================================================
// Get and Store Default Values for position and orientation
//=============================================================================
bool initializePosition(std::ifstream& file, std::string& next_value)
{

  string parameter;
  float value = 0.0;

  getNextToken( file, next_value );

  // Go through file and find MINOR tags and store the data next to it in its
  // respective variable.
  while( file )
  {
    if ( next_value[0] == '#' )
    {
      char buffer[1024];
      file.getline(buffer, sizeof(buffer), '\n');
    }

    parameter = next_value;
    getNextToken(file, next_value);
    value = static_cast<float>(atof(next_value.c_str()));

    if( !parameter.compare("LAT") )
    {
    	refPos.LAT = value;
    }
    else if( !parameter.compare("LON") )
    {
    	refPos.LON = value;
    }
    else if( !parameter.compare("ALT") )
    {
    	refPos.ALT = value;
    }
    else if( !parameter.compare("HEADING") )
    {
    	initial_heading = value;
    }
    else
    {
      std::cout << "Did not find a parameter in position. Exiting!"
    		  << std::endl;
      return false;
    }
    getNextToken(file, next_value);
  }
  return true;
}

//=============================================================================
// Extracts a token from the file, excluding the remainder of any line from a
// comment ('#'), on.
//=============================================================================
bool getNextToken(ifstream& file, string& next_token)
{
  if ( !file )
  {
    return false;
  }

  file >> next_token;

  while ( file && (next_token[0] == '#') )
  {
    // This is a comment, not a token; toss the rest of the line and keep
    // looking
    static char buffer[1024];
    file.getline(buffer, sizeof(buffer), '\n');
    file >> next_token;
  }

  if ( !file )
  {
    return false;
  }

  makeStringUppercase(next_token);

  return true;
}

inline string& makeStringUppercase(string& s)
{
	for(string::size_type i=0; i<s.length(); i++)
	{
		s.at(i) = char(toupper(s[i]));
	}
	return s;
}


#endif /* ICREADER_H_ */
