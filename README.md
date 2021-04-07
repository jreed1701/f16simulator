# JPRSIM - F-16 C++ Flight Dynamics Simulator

**About:**

JPRSIM is a Rigid Body Flight Dynamics simulator with 6 degrees of freedom. The simulator uses a component built up method to compute forces of the aircraft model.  The aircraft model itself is an F-16 Fighting Falcon represented by a series of aerodynamic tables, which contain values collected from wind tunnel data.  Keep in mind the software was built to operate a simulator and is not simply a simulation.  

**Features**

The simulator iteself is written in C++ and is verified to compile on Ubuntu 18.04 with gcc version 7.5 using the C++11 standard.

The original simulator had the following features that are no longer supported but are avaialable for those interested as a reference.

* The "jprsim" binary executable to simulate an F-16 aircraft.
* Logging Ability for aircraft state data.
* Ability to read an configuration input file.
* A UDP socket output to a FlightGear display port for visuals.
* A UDP socket input for flight inceptor (joystick/throttle/rudder) pedals.
* A Matlab User Interface to serve as a simulator interface.
* A Matlab tool that has the ability to compute trim conditions to later be used as initial conditions.
* A LabView Component to operate a National Instruments data collection for joystick inputs.

**Limitations**

* This simulation is a rigid body dynamics simulator so it does not model changes in moments due to inertia. For example, fuel burn.
* The lattitude/longitude computations assume a flat earth model, therefore simulations where the aircraft travels 50+ nautical miles away from start will deviate.
* The zero foot altitude is not aligned with any visual database, so it is possible to go "underground" when using visuals.

**Versions**

1.0 - This is the code originally submitted. The reference folder contains older software no longer supported.

**Future work**

The author is not actively working on this as it is a older graduate school project, however, to maintain practice with C++ the following ideas would help make a nice version 2.0.

* Update software design to be more object oriented. The author was less experienced at the time of original development and the code follows a more procedural pattern.
* Update code to have a different method of inputting controls. The original simulator used a custom platform with a LabView project to read in inputs. A GUI Cockpit or keyboard input method may server as a suitable replacement.
* Update the code for quality and apply DevOps methods to the codebase.
* Dockerize this software.

**Reuse**

Anyone has my permission to use this software as they wish and contribute. I can't promise that I will be adding requested features, if any, but don't mind discussing things via GH Issues.
