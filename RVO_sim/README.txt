Pedestrian Simulation Using RVO2 Simulator
/*******************************************/
Written by: C. Elder
Date: 04/14/2014
/*******************************************/

Table of Contents
(1) Introduction
(2) Installation
(3) Use Instructions
(4) Example Description
(5) Acknowledgements

/**************************************************************************************/
(1) Introduction
/**************************************************************************************/
(a) Description
	This program creates a (hopefully easy) way to use the RVO object interaction
	library to generate and record object tracks through the fields of view of cameras
	in a pre-determined scene.

	The initial purpose of this program was to generate data for automatic camera
	network location and orientation determination for a CS231a project at Stanford
	University, Winter Quarter 2014.

(b) Included files
   	README.txt	- This file
   Main Files
	sim.cpp		- Main function, performs simulation based on description given
			  in input file
	parser.hpp	- Header file for parser.cpp
	parser.cpp	- Source file to parse input file for scene description
	Makefile	- Makefile to compile simulation
	RVO2/		- RVO Library Source files
   Output Files
	data/*		- See Section 3(c)   
   Example Files
	example.txt	- Example scene description input file (see Section 4)
	exampleScene.jpg- Picture of scene described in example.txt
	displayExampleScene.m - File which created exampleScene.jpg
   Additional Files
	show_tracks.m	- Rudimentary file to display tracks from output files in scene.
			  Will require work to make more general.


/**************************************************************************************/
(2) Installation
/**************************************************************************************/
(a) RVO2 Library Installation - RVO2 Library C++ v2.0.1
The RVO2 Library source has been included in the folder containing this README.txt file.
Please use ./RVO2 as $RVO_ROOT when installing below.

From RVO2 library installation instructions:
*We assume that you have downloaded RVO2 Library and unpacked the ZIP archive to a path
*$RVO_ROOT.
*
*Apple Xcode 3.x
*Open $RVO_ROOT/RVO.xcodeproj and select the RVO target and a configuration (Debug or 
*Release). A framework RVO.framework will be built in the default build directory, e.g. 
*$RVO_ROOT/build/Release.
*
*CMake
*Create and switch to your chosen build directory, e.g. $RVO_ROOT/build. Run cmake inside 
*the build directory on the source directory, e.g. cmake $RVO_ROOT/src. Build files for 
*the default generator for your platform will be generated in the build directory.
*
*GNU Make
*Switch to the source directory $RVO_ROOT/src and run make. Public header files (RVO.h,
*RVOSimulator.h, and Vector2.h) will be copied to the $RVO_ROOT/include directory and a 
*static library libRVO.a will be compiled into the $RVO_ROOT/lib directory.
*
*Microsoft Visual Studio 2008
*Open $RVO_ROOT/RVO.sln and select the RVO project and a configuration (Debug, ReleaseST, 
*or ReleaseMT). Public header files (RVO.h, RVOSimulator.h, and Vector2.h) will be copied 
*to the $RVO_ROOT/include directory and a static library, e.g. RVO.lib, will be compiled 
*into the $RVO_ROOT/lib directory.

(b) Simulation Files - sim.cpp
<*nix>
If $RVO_ROOT above was ./RVO2, then the included Makefile will make the simulation (see
below)
Otherwise, adjust Makefile to point at RVO libraries

<Windows>
Make sure to link the libraries and files listed in the included Makefile when you
compile in Visual Studio

/**************************************************************************************/
(3) Use Instructions
/**************************************************************************************/
(a) *** Command Line Use ***
<*nix> (Requires Make and gcc-4.2 (?) or higher)
- Clean Previous 
	> make clean
- Make Simulator
	> make sim
- Run Simulator
	This reads scene and default agent information from SetupFilename and writes
	the ground truth tracks of the agents to ./data/<OutputFileBase>.csv and the
	agents’ paths across each camera, in the camera’s reference coordinate system,
	to ./data/<OutputFileBase>_cam<CamNum>.csv
 
	> ./sim <SetupFilename> <OutputFileBase>

	> ./sim <SetupFilename>
	-> uses “out” as OutputFileBase

	> ./sim
	-> Uses “example.txt” as SetupFilename and “out” as OutputFileBase

(b) *** Scene/Agent Description File (“SetupFilename”) ***
The Scene/Agent setup file is comprised of 6 sections, all of which are required. In
addition, the parser (in parser.cpp) is fairly robust to errors, but will yell at you if
you are too far from the intended syntax for each section. I apologize in advance for any
trouble you might have with formatting. See “example.txt” for the correct syntax.

Sections are delineated by a ‘#’ symbol, after which anything can be on the same line.

Section 0 - Sim Details
Desc: This section captures the simulation time step and length in seconds
	- NOTE: This section is not forgiving to out of order parameters
Syntax:	Timestep=<interval between simulation steps in seconds>
	LengthofSim=<length of time to simulate in seconds>

Section 1 - Agent Details
Desc: This section captures the size and max speed of agents, as well as the number of 
	agents moving from their entrance to their exit at a given time.
	- NOTE: This section is not forgiving to out of order parameters
Syntax:	radius = <agent radius in meters>
	maxSpeed=<maximum speed of an agent in meters/second>
	numActiveAgents= <number of agents active in sim in a single timestep>

Section 2 - Scene Description
Desc: This section captures the obstacles or walls of the scene. Agents will avoid these
	obstacles when moving from entrance to exit.
	- NOTE: Obstacles are polygons defined by their vertices in counter-clockwise
	 order.
	- Simulation will connect last vertex to first vertex to make a solid shape.
	- One obstacle per line.
	- Below are L+1 Objects; Obj1 has M+1 vertices, Obj2 N+1 vertices, ObjL K+1 
	 vertices 
Syntax:	(x_obs0_v0,y_obs0_v0);(x_obs0_v1,y_obs0_v1);<…>;(x_obs0_vM,y_obs0_vM);
	(x_obs1_v0,y_obs1_v0);(x_obs1_v1,y_obs1_v1);<…>;(x_obs1_vN,y_obs1_vN);
	<…>
	(x_obsL_v0,y_obsL_v0);(x_obsL_v1,y_obsL_v1);<…>;(x_obsL_vK,y_obsL_vK);

Section 3 - Agent Entrance/Exit Lines
Desc: This section captures all possible lines on which agents can start and end.
	- Lines are specified by endpoints, 1 entrance and 2 exits per line
	- Agents will start from a point on an entrance line with uniform probability
	- The two exits per line are a pair, with an agent passing through exit 1 before
	 heading to exit 2
	- The first exit per line should be towards the middle of the map, thus avoiding
	 agents getting stuck heading toward their final exit
	- The second exit per line should be “off the map” so that multiple agents can
	 run through the simulation and stop at their 2nd exit for the rest of the sim
	 without hindering the progress of more recent agents
	- Below are N+1 entrance/exit lines
Syntax: 
(x0ent0,y0ent0);(x1ent0,y1ent0);(x0ex0,y0ex0);(x1ex0,y1ex0);(x0ex0,y0ex0);(x1ex0,y1ex0);
(x0ent1,y0ent1);(x1ent1,y1ent1);(x0ex1,y0ex1);(x1ex1,y1ex1);(x0ex1,y0ex1);(x1ex1,y1ex1);
<…>
(x0entN,y0entN);(x1entN,y1entN);(x0exN,y0exN);(x1exN,y1exN);(x0exN,y0exN);(x1exN,y1exN);

Section 4 - Entrance->Exit Probabilities
Desc: This section captures the probability of an agent assigned to entrance i to exit at
	exit(s) j.
	- NOTE: This must be a square matrix and rows must sum to 1
	- Agents are assigned an entrance with uniform probability
	- Below are N+1 Entrances and Exit Pairs; Entrances are rows, exits columns
Syntax: P(ex0|ent0),P(ex1|ent0),<…>,P(exN|ent0);
	P(ex0|ent1),P(ex1|ent1),<…>,P(exN|ent1);
	<…>
	P(ex0|entN),P(ex1|entN),<…>,P(exN|entN);

Section 5 - Camera Locations
Desc: This section captures each camera’s location and orientation in the global coords
	- Below are N+1 cameras
	- Theta is angle (in degrees) counter-clockwise off of x-axis
Syntax: x_cam0,y_cam0,theta_cam0;
	x_cam1,y_cam1,theta_cam1;
	<…>
	x_camN,y_camN,theta_camN;

Additional Notes
- Whitespace is ignored, be it a blank line or extra spaces/tabs between numbers

(c) *** Output File Formats ***
<OutputFileBase>.csv
Desc: This file captures the ground truth of each track in the simulator until it reaches
	its second exit
	- Each line of the file represents a different track, in chronological starting
	 order
	- Each point of each track consists of an (x,y) coordinate at global time t
	- Shorter tracks are padded with -1 so that all tracks have the same length in
	 the csv file
	- Below we have N+1 tracks; tr0 has M+1 points; tr1 L+1 points; trN K+1 points;
	 M < L < K
Syntax: 
0 <tr#> <numPtsInTr> x0tr0 y0tr0 t0tr0 x1tr0 y1tr0 t1tr0 <…> xMtr0 yMtr0 tMtr0 -1 <…> -1
0 <tr#> <numPtsInTr> x0tr1 y0tr1 t0tr1 x1tr1 y1tr1 t1tr1 <…> xLtr1 yLtr1 tLtr1 -1 <…> -1
<…>
0 <tr#> <numPtsInTr> x0trN y0trN t0trN x1trN y1trN t1trN <…> xKtrN yKtrN tKtrN


<OutputFileBase>_cam<CamNum>.csv
Desc: This file captures the tracks which cross camera <CamNum>’s field of view
	- Each line represents the position (in camera coords) of a track at time t
	- Lines are organized in blocks by track number, and are chronologically ordered
	 within each block
	- Chronological order is not preserved between blocks
	- Below are N+1 tracks; tr0 has M+1 points; tr1 L+1 points; trN K+1 points
Syntax: x0tr0 y0tr0 t0tr0 <tr# for tr0>
	x1tr0 y0tr0 t0tr0 <tr# for tr0>
	<…>
	xMtr0 yMtr0 t0tr0 <tr# for tr0>
	x0tr1 y0tr1 t0tr1 <tr# for tr1>
	x1tr1 y1tr1 t1tr1 <tr# for tr1>
	<…>
	xLtr1 yLtr1 tLtr1 <tr# for tr1>
	<…>
	x0trN y0trN t0trN <tr# for trN>
	x1trN y1trN t1trN <tr# for trN>
	<…>
	xKtrN yKtrN tKtrN <tr# for trN>

	
/**************************************************************************************/
(4) Example Description
/**************************************************************************************/
The example in example.txt is for a 100 m long by 16 m wide hallway with 8 
entrance/exits, one at either end, three equally spaced along the top, and the last
three equally spaced along the bottom. Top and bottom entrance/exits are 8 m wide.

See exampleScene.jpg for a visual of the scene. 
Entrance lines are green, 1st exit lines are yellow, 2nd exit lines are red.
Obstacles are black.
Camera FOVs are magenta.
Area in which agents can move is blue.


/**************************************************************************************/
(5) Acknowledgements
/**************************************************************************************/
- Simulation accomplished using “RVO2 Library: Reciprocal Collision Avoidance for Real-
Time Multi-Agent Simulation” by Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. 
Lin, and Dinesh Manocha from the Department of Computer Science, University of North 
Carolina at Chapel Hill <http://gamma.cs.unc.edu/RVO2/>

