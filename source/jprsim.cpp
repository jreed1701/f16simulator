/*jprsim.cpp : Defines the entry point for the console application.
Author: Joshua Reed
Date: 2/10/2013
Description: Custom C code for an F-16 flight simulator
*/


// Built in Headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <netinet/in.h>

// Custom header files
#include "interp.h"
#include "jprsim.h"
#include "constants.h"

// Declare X0 and U0 here so the IC reader can see them in its scope.
X X0; U U0;
double initial_heading;
GEO  refPos;
double HERTZ_COMMAND = 0.0;
#include "icreader.h"

// Flight Gear Header(s)
#include "net_fdm.hxx"

//#define STEP    0.01         // 5 Hz
//#define UPDATE_PERIOD 10000 // microseconds
#define MAX_RUNTIME 100000            // seconds

// Create objects needed to support UDP transmission
// IP and port where FG is listening
//struct sockaddr_in echoServAddr;
char* fg_ip = "127.0.0.1";
int   fg_port = 5500;
int   fg_socket = 0;

// Control Inceptor UDP
char* localhost = "127.0.0.1";
int inceptor_port = 5505;
int inceptor_socket = 0;

// Global Variables
X curX, xDot;
U curU, lastU;
data forces;
GEO curPos, zeroPos;
double STEP = 0.01;           // seconds
double UPDATE_PERIOD = 10000; // micro seconds

// Sim control bools
bool HOLD = true;
bool OPERATE = false;
bool RESET = false;

double*** cax_alfa_beta_hrzt_tab;
double*** caz_alfa_beta_hrzt_tab;
double*** cal_alfa_beta_hrzt_tab;
double*** cam_alfa_beta_hrzt_tab;
double*** can_alfa_beta_hrzt_tab;
double*** thst_alt_mach_powr_tab;

double heading;
double step_heading;
double lastX = 0;
double lastY = 0;


using namespace std;

int main(int argc, char* argv[])
{
    // Simulation variables
    FGNetFDM fdm;
    memset(&fdm,0,sizeof(fdm));
    memset(&U0,0,sizeof(U0));
    fdm.version = htonl(FG_NET_FDM_VERSION);
    fdm.visibility = htonf(5000.0);
    FILE *fid;

    double time = 0.0;

    // Open a log file
    fid = fopen("JPRSIM_LOG.txt","w");

    // Read command line arguments and act on them
    parseCommandLine(argc, argv);

    // Initialize all simulation resources and initial conditions
    initializeSimulation();

    while( (time < MAX_RUNTIME)  &&  ( HOLD || OPERATE ) ){

        // Sleep for step interval
        usleep(UPDATE_PERIOD);

        // Update inceptor positions
        curU = proccessInceptors(inceptor_socket, inceptor_port, lastU);

        if( curU.MODE == MODE_RESET)
        {
            cout << "Reseting Time and Log file" << endl;
            fclose(fid);
            fid = fopen("JPRSIM_LOG.txt","w");
            time = 0;
        }

        // process current mode
        modeProcessing(curU.MODE);

        // Check for exit command, Break loop if received.
        if( curU.MODE == MODE_EXIT)
        {
            cout << "Terminate Command Received. Exiting.." << endl;
            break;
        }

        // While the simulation is in HOLD mode the user can move the the inceptors
        // to where they need to be.  This is preferable because not doing so will
        // cause the aircraft to fly awkwardly away. Remember you want to start off
        // in a trim condition.
        if( HOLD && !OPERATE)
        {
           inceptorPositionCheck();
        }


        if( OPERATE )
        {
            // Calculate RK4
            curX = RK4(curX,curU,STEP,forces,cax_alfa_beta_hrzt_tab,caz_alfa_beta_hrzt_tab,cal_alfa_beta_hrzt_tab,
            cam_alfa_beta_hrzt_tab,can_alfa_beta_hrzt_tab,thst_alt_mach_powr_tab);

            // Get xDot from current state
            xDot = getEOM (curX,curU,forces,cax_alfa_beta_hrzt_tab,caz_alfa_beta_hrzt_tab,cal_alfa_beta_hrzt_tab,
                    cam_alfa_beta_hrzt_tab,can_alfa_beta_hrzt_tab,thst_alt_mach_powr_tab);

            // Calculate aircraft heading
            step_heading = atan2( xDot.y, xDot.x ) * R2D;

            heading = initial_heading + step_heading;

            // Get Latitude / Longitude / Altitude
            refPos.ALT = -1*curX.z;

            curPos = moveAircraftPosition( calcRange(lastX - curX.x, lastY - curX.y), heading, refPos );

            // Save current position for next loop
            lastX = curX.x;
            lastY = curX.y;
            refPos = curPos;

            // Write data to log file
            write_data(fid,curX, curU, curPos, time);

            // Increment time step
            time += STEP;
        }
        // Save current inceptor position for next loop
        lastU = curU;

        // Export aircraft state and position to FlightGear
        export2FG(curX, xDot, curU, curPos, heading, fg_socket, fg_ip, fg_port, &fdm);

    }
    fclose(fid);
    printf("Exited with time = %f\n",time);
    return EXIT_SUCCESS;
}

void modeProcessing( int mode)
{
    if( mode == 1)
    {
        OPERATE = true;
        HOLD = false;
        RESET = false;
    }
    else if( mode == 2)
    {
        OPERATE = false;
        HOLD = true;
        RESET = false;
    }
    else if( mode == 3)
    {
        OPERATE = false;
        HOLD = true;
        RESET = true;
        doReset();
    }
}

void doReset()
{
    resetInitialization();
    refPos = curPos;
    lastX = curX.x;
    lastY = curX.y;
    RESET = false;
}

void resetInitialization()
{
    heading = initial_heading;

    curPos = zeroPos;
    //refPos = curPos;

    X0.x = 0.0;
    X0.y = 0.0;
    X0.z = -1.0*zeroPos.ALT;

    X0.u = X0.vt*cos(X0.alfa*D2R)*cos(X0.beta*D2R);
    X0.v = X0.vt*sin(X0.beta*D2R);
    X0.w = X0.vt*sin(X0.alfa*D2R)*cos(X0.beta*D2R);
    X0.p = 0.0;
    X0.q =0.0;
    X0.r = 0.0;
    X0.north = 0.0;
    X0.east  = 0.0;
    X0.down  = 0.0;

    U0.MODE = 0;

    curX = X0; // Set RK4 initial state.
}

void initializeSimulation()
{
    // Create UDP Client and Server
    fg_socket = socketUDP_client_init();
    inceptor_socket = socketUDP_init(localhost, inceptor_port);

    // If the user specified a refresh rate for the sim, then this
    // value will be non zero.
    if(HERTZ_COMMAND)
    {
        STEP = 1 / HERTZ_COMMAND;
        UPDATE_PERIOD = STEP * SEC2MICROSEC;
    }

    // Save a copy of the first position
    zeroPos = refPos;

    heading = initial_heading;

    curPos = refPos;

    // initialize all 3d tables
    cax_alfa_beta_hrzt_tab = initialize5(cax_alfa_beta_hrzt_m25_tab, cax_alfa_beta_hrzt_m10_tab, cax_alfa_beta_hrzt_0_tab, 
        cax_alfa_beta_hrzt_p10_tab, cax_alfa_beta_hrzt_p25_tab);
    caz_alfa_beta_hrzt_tab = initialize5(caz_alfa_beta_hrzt_m25_tab, caz_alfa_beta_hrzt_m10_tab, caz_alfa_beta_hrzt_0_tab,
        caz_alfa_beta_hrzt_p10_tab, caz_alfa_beta_hrzt_p25_tab);
    cal_alfa_beta_hrzt_tab = initialize3(cal_alfa_beta_hrzt_m25_tab, cal_alfa_beta_hrzt_0_tab, cal_alfa_beta_hrzt_p25_tab);
    cam_alfa_beta_hrzt_tab = initialize5(cam_alfa_beta_hrzt_m25_tab, cam_alfa_beta_hrzt_m10_tab, cam_alfa_beta_hrzt_0_tab,
        cam_alfa_beta_hrzt_p10_tab, cam_alfa_beta_hrzt_p25_tab);
    can_alfa_beta_hrzt_tab = initialize3(can_alfa_beta_hrzt_m25_tab, can_alfa_beta_hrzt_0_tab, can_alfa_beta_hrzt_p25_tab);
    thst_alt_mach_powr_tab = initialize3Prop(thst_alt_mach_powr_idle_tab, thst_alt_mach_powr_mil_tab, thst_alt_mach_powr_max_tab);

    // any other initializations required
    forces.delta   = (ixx*iyy*izz)-(ixx*pow(iyz,2.0))-(iyy*pow(izx,2.0))-(izz*pow(ixy,2.0))-(2.0*ixy*iyz*izx);
    forces.ixx_inv = (iyy*izz-pow(iyz,2.0))/forces.delta;
    forces.ixy_inv = (izz*ixy+izx*iyz)/forces.delta; //% neg
    forces.iyy_inv = (ixx*izz-pow(izx,2.0))/forces.delta;
    forces.iyz_inv = (ixx*iyz+ixy*izx)/forces.delta; //% neg
    forces.izz_inv = (ixx*iyy-pow(ixy,2.0))/forces.delta;
    forces.izx_inv = (iyy*izx+ixy*iyz)/forces.delta; //% neg

    X0.x = 0.0;
    X0.y = 0.0;
    X0.z = -1.0*refPos.ALT;
    X0.u = X0.vt*cos(X0.alfa*D2R)*cos(X0.beta*D2R);
    X0.v = X0.vt*sin(X0.beta*D2R);
    X0.w = X0.vt*sin(X0.alfa*D2R)*cos(X0.beta*D2R);
    X0.p = 0.0;
    X0.q =0.0;
    X0.r = 0.0;
    X0.north = 0.0;  // This origin will be placed at some initial ECEF coordinate
    X0.east  = 0.0;
    X0.down  = 0.0;

    U0.MODE = 0;

    curX = X0; // Set RK4 initial state.

}

/*
 * This function makes sure that the user moves the inceptors to the
 * Required position while in hold before going to operate.
 */
void inceptorPositionCheck()
{
    std::string th_msg;
    std::string flap_msg;
    std::string SpdBr_msg;
    std::string Pitch_msg;
    std::string Roll_msg;
    std::string rud_msg;

    if( withinEpsilonOf(curU.Th,U0.Th,0.005))
    {
        th_msg = " Matched!";
    }
    else
    {
        th_msg = " Adjust Throttle.";
    }

    if( withinEpsilonOf(curU.Flap,U0.Flap,0.005))
    {
        flap_msg = " Matched!";
    }
    else
    {
        flap_msg = " Adjust Flaps.";
    }

    if( withinEpsilonOf(curU.SpdBr,U0.SpdBr,0.005))
    {
        SpdBr_msg = " Matched!";
    }
    else
    {
        SpdBr_msg = " Adjust Speed Brake.";
    }

    if( withinEpsilonOf(curU.delH,U0.delH,0.005))
    {
        Pitch_msg = " Matched!";
    }
    else
    {
        Pitch_msg = " Adjust Pitch Trim.";
    }

    if( withinEpsilonOf(curU.delA,U0.delA,0.005))
    {
        Roll_msg = " Matched!";
    }
    else
    {
        Roll_msg = " Adjust Aileron Trim.";
    }

    if( withinEpsilonOf(curU.delR,U0.delR,0.005))
    {
        rud_msg = " Matched!";
    }
    else
    {
        rud_msg = " Adjust Rudder Trim.";
    }

    std::cout << "################################" << std::endl;
    std::cout << "DelH Initial:   " << U0.delH
              << " Current: "       << curU.delH
              << Pitch_msg << std::endl;
    std::cout << "DelA Initial:   " << U0.delA
              << " Current: "       << curU.delA
              << Roll_msg  << std::endl;
    std::cout << "DelR Initial:   " << U0.delR
              << " Current: "       << curU.delR
              << rud_msg   << std::endl;
    std::cout << "Throttle Init:  " << U0.Th
              << " Current: "       << curU.Th
              << th_msg << std::endl;
    std::cout << "Flapt Initial:  " << U0.Flap
              << " Current: "       << curU.Flap
              << flap_msg << std::endl;
    std::cout << "Spd Brake Init: " << U0.SpdBr
              << " Current: "       << curU.SpdBr
              << SpdBr_msg << std::endl;
    std::cout << "################################" << std::endl;
}
