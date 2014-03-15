#ifndef __PARSER_H_INCLUDED__
#define __PARSER_H_INCLUDED__

#include <vector>
#include <string>
#include "RVO.h"

using namespace std;
/* Function to get values of certain details of Sim setup and Agent default
 *  Input(s):
 *      string line - line to be parsed for value
 *          Line should be of the format: <detailName>=<value>
 *  Output(s):
 *      float value - value of Sim or Agent detail
 *      int         - 1 if successful, 0 if failed
 */
int getSimAgentLineDetails(string line,
                           float & value);


/* Function to get verticies of the sim Bounding Box or an obstacle
 *  Input(s):
 *      string line - line to be parsed for value
 *          Line should be of the format: (<x0>,<y0>);(<x1>,<y1>);...
 *          NOTE: For Bounding Box, vertices must be listed in clockwise format
 *                For obstacles, vertices must be listed in counterclockwise format
 *          THIS FUNCTION DOES NOT CHECK CLOCKWISE/COUNTERCLOCKWISE
 *  Output(s):
 *      vector< vector<float> > vertices - 2D array of vertices
 *      int         - 1 if successful, 0 if failed
 */
int getSceneDetails(string line,
                    vector< vector<float> >& vertices);


/* Function to get transition probabilities of an Agent going from entrance i to one
 * of the possible exits
 *  Input(s):
 *      string line - line to be parsed for value
 *          Line should be of the format: in_i_out_0,in_i_out1,in_i_out2,...
 *  Output(s):
 *      vector<float> singleEntranceProbs - array of transition probabilites
 *      int         - 1 if successful, 0 if failed
 */
int getEntranceProbabilities(string line,
                             vector<float>& singleEntranceProbs);


/* Function to get the location of a camera in world coordinates
 *  Input(s):
 *      string line - line to be parsed for value
 *          Line should be of the format: x,y,theta
 *  Output(s):
 *      vector<float> singleCameraLocation - location of a camera in world
 *                                           coordinates
 *      int                               - 0 if successful, 1 if failed
 */
int getCameraLocations(string line,
                       vector<float>& singleCameraLocation);



/* Function to update simulator with details
 *  Input(s):
 *      vector<string> lines    - vector of lines of details read from file
 *      RVOSimulator* sim       - simulator object to be updated
 *  Output(s):
 *      int               - 1 if successful, 0 if failed
 *      float lengthOfSim - length of simulation time (in seconds)
 */
int processSimDetails (vector<string> lines,
                       RVO::RVOSimulator* sim,
                       float& lengthOfSim);


/* Function to update simulator with default agent details
 *  Input(s):
 *      vector<string> lines    - vector of lines of details read from file
 *      RVOSimulator* sim       - simulator object to be updated
 *  Output(s):
 *      int                 - 1 if successful, 0 if failed
 *      int numActiveAgents - number of agents to have moving in the simulation at a 
 *                            single timestep
 */
int processAgentDetails(vector<string> lines,
                        RVO::RVOSimulator* sim,
                        int& numActiveAgents);


/* Function to update simulator with scene description
 *  Input(s):
 *      vector<string> obstacles    - vector of lines of coordinates read from file
 *      RVOSimulator* sim           - simulator object to be updated
 *  Output(s):
 *      int         - 1 if successful, 0 if failed
 */
int processScene(vector<string> obstacles,
                 RVO::RVOSimulator* sim);


/* Function to get possible entrances/exits for agents
 *  Input(s):
 *      vector<string> lines    - vector of lines of coordinates read from file
 *  Output(s):
 *      int                             - 1 if successful, 0 if failed
 *      vector<vector<RVO::Vector2>> entrances - vector containing vectors describing
 *                                        entrance/exit lines by endpoints
 *                                        (i.e. [[(x0_0,y0_0),(x1_0,y1_0)],
 *                                               [(x0_1,y0_1),(x1_1,y1_1)], ...
 *                                              ])
 */
int processAgentEntrancesExits(vector<string> lines,
                               vector< vector<RVO::Vector2> >& entrances);


/* Function to get transition probabilities for entrance->exit paths
 *  Input(s):
 *      vector<string> lines    - vector of lines of coordinates read from file
 *  Output(s):
 *      int                             - 1 if successful, 0 if failed
 *      vector<vector<float>> probs - vector containing entrance->exit probabilites
 *                                        (i.e. [[in0out0,in0out1, ...],
 *                                               [in1out0,in1out1, ...], ...
 *                                              ]
 */
int processEntranceProbabilities(vector<string> lines,
                                 vector< vector<float> >& probs);


/* Function to get locations of cameras in world coordinates
 *  Input(s):
 *      vector<string> lines    - vector of lines of coordinates read from file
 *  Output(s):
 *      int                             - 1 if successful, 0 if failed
 *      vector<vector<float>> cams - vector containing camera locations
 *                                        (i.e. [[x0,y0,theta0],
 *                                               [x1,y1,theta1], ...
 *                                              ]
 */
int processCameraLocations(vector<string> lines,
                           vector< vector<float> >& cams);

#endif  // __PARSER_H_INCLUDED__


