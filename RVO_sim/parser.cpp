#include <iostream>
//#include <string>
#include <fstream>
//#include <vector>
#include <exception>
//#include "RVO.h"

#include "parser.hpp"

//using namespace std;

/* Function to get values of certain details of Sim setup and Agent default
 *  Input(s):
 *      string line - line to be parsed for value
 *          Line should be of the format: <detailName>=<value>
 *  Output(s):
 *      float value - value of Sim or Agent detail
 *      int         - 0 if successful, 1 if failed
 */
int getSimAgentLineDetails(string line, float & value)
{
    // Find index of '='
    size_t ind = line.find_first_of("=");
    
    // Check that '=' is in string
    if (ind == string::npos) {
        cout << "ERROR: Improper Formatting of Sim Details" << endl;
        cout << "Sim Details Proper Format: <detailName>=<value>";
        return 1;
    }
    
    // Take substring after '='
    string value_s = line.substr(ind+1);
    
    // Convert to float
    try {
        value = stof(value_s);
    } catch (exception& e) {
        cout << "Unable to Load SimLineDetails" << endl;
        cout << "Exception: " << e.what() << endl;
        return 1;
    }
    
    // Return
    return 0;
}


/* Function to get verticies of the sim Bounding Box or an obstacle
 *  Input(s):
 *      string line - line to be parsed for value
 *          Line should be of the format: (<x0>,<y0>);(<x1>,<y1>);...
 *          NOTE: For Bounding Box, vertices must be listed in clockwise format
 *                For obstacles, vertices must be listed in counterclockwise format
 *          THIS FUNCTION DOES NOT CHECK CLOCKWISE/COUNTERCLOCKWISE
 *  Output(s):
 *      vector< vector<float> > vertices - 2D array of vertices
 *      int                              - 0 if successful, 1 if failed
 */
int getSceneDetails(string line, vector< vector<float> >& vertices)
{
    // Initialize
    size_t ind = 0;
    size_t xStart = 0;
    size_t yStart = 0;
    size_t xLen = 0;
    size_t yLen = 0;
    size_t validX = 0;
    size_t validY = 0;
    char thisChar;
    vector<float> singleCoordinate;
    float xCoord, yCoord;
    string delimiters = "(,);";
    
    // Loop through each character of the string
    while (ind < line.size()) {
        // Get this character
        thisChar = line[ind];
        
        // Starting X Coordinate
        if ((thisChar == '(') && (validX == 0)) {
            validX = 1;
            xStart = ind+1;
        } else
            // Still Recording X Coordinate
            if ((delimiters.find(thisChar) == string::npos) && (validX == 1)) {
                xLen ++;
        } else
            // Hit End of X Coordinate - Write to x_coord, Start Y
            if ((thisChar == ',') && (validX == 1)) {
                // Convert to float
                try {
                    xCoord = stof(line.substr(xStart,xLen));
                } catch (exception& e) {
                    cout << "ERROR - Unable to Load Scene Details" << endl;
                    cout << "Exception: " << e.what() << endl;
                    return 1;
                }
                // Set valid_x to 2
                validX = 2;
                // Start Y Recording
                validY = 1;
                yStart = ind+1;
        } else
            // Still Recording Y Coordinate
            if ((delimiters.find(thisChar) == string::npos) && (validY == 1)) {
                yLen ++;
        } else
            // Hit End of Y Coordinate - Write to y_coord
            if ((thisChar == ')') && (validY == 1)) {
                // Convert to float
                try {
                    yCoord = stof(line.substr(yStart,yLen));
                } catch (exception& e) {
                    cout << "ERROR - Unable to Load Scene Details" << endl;
                    cout << "Exception: " << e.what() << endl;
                    return 1;
                }
                // Set valid_y to 2
                validY = 2;
        } else
            // Hit End of Coordinate - Write Coordinate to vertices
            if ((validX == 2) && (validY == 2)) {
                // Write to singleCoordinate
                singleCoordinate.push_back(xCoord);
                singleCoordinate.push_back(yCoord);

                // Write vertice to vertices
                vertices.push_back(singleCoordinate);
                
                // Clear singleCoordinate
                singleCoordinate.clear();
                
                // Set Flags to 0
                validX = 0;
                validY = 0;
                xLen = 0;
                yLen = 0;
                
        } else
            // Skip Extra Characters between coordinates
            if ((validX == 0) && (validY == 0)) {
                
        } else {
            // Must be an error
            cout << "ERROR - Invalid character sequence in GetSceneData" << endl;
            return 1;
        }
        
        // Increment character index
        ind ++;
    }
    
    return 0;
}


/* Function to get transition probabilities of an Agent going from entrance i to one
 * of the possible exits
 *  Input(s):
 *      string line - line to be parsed for value
 *          Line should be of the format: in_i_out_0,in_i_out1,in_i_out2,...
 *  Output(s):
 *      vector<float> singleEntranceProbs - array of transition probabilites
 *      int                               - 0 if successful, 1 if failed
 */
int getEntranceProbabilities(string line,
                             vector<float>& singleEntranceProbs)
{
    // Initialize
    size_t ind = 0;
    size_t numStart = 0;
    size_t numLen = 0;
    size_t inNum = 0;
    char thisChar;
    float thisNum;
    string delimiters = ",;";
    string nums = "0123456789.";
    
    // Loop through each character of the string
    while (ind < line.size()) {
        // Get this character
        thisChar = line[ind];
        
        // Start recording number
        if ((inNum == 0) && (nums.find(thisChar) != string::npos)) {
            numStart = ind;
            numLen ++;
            inNum = 1;
        } else
            // Continue Recording Number
            if ((inNum == 1) && (nums.find(thisChar) != string::npos)) {
                numLen ++;
        } else
            // Finish Recording Number
            if ((inNum == 1) && (delimiters.find(thisChar) != string::npos)) {
                // Convert to float
                try {
                    thisNum = stof(line.substr(numStart,numLen));
                } catch (exception& e) {
                    cout << "ERROR - Unable to Load Scene Details" << endl;
                    cout << "Exception: " << e.what() << endl;
                    return 1;
                }
                
                // Add probability to vector
                singleEntranceProbs.push_back(thisNum);
                
                // Reinitialize
                numLen = 0;
                inNum = 0;
        } else
            // Skip extra characters between numbers
            if (inNum == 0) {
                
        } else {
            // Must be an error
            cout << "ERROR - Invalid character sequence in GetEntranceProbabilities" << endl;
            return 1;
        }
        
        // Increment character index
        ind++;
    }
    
    // Return
    return 0;
}


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
                       vector<float>& singleCameraLocation)
{
    // Initialize
    size_t ind = 0;
    size_t numStart = 0;
    size_t numLen = 0;
    size_t inNum = 0;
    char thisChar;
    float thisNum;
    string delimiters = ",;";
    string nums = "0123456789.";
    
    // Loop through each character of the string
    while (ind < line.size()) {
        // Get this character
        thisChar = line[ind];
        
        // Start recording number
        if ((inNum == 0) && (nums.find(thisChar) != string::npos)) {
            numStart = ind;
            numLen ++;
            inNum = 1;
        } else
            // Continue Recording Number
            if ((inNum == 1) && (nums.find(thisChar) != string::npos)) {
                numLen ++;
            } else
                // Finish Recording Number
                if ((inNum == 1) && (delimiters.find(thisChar) != string::npos)) {
                    // Convert to float
                    try {
                        thisNum = stof(line.substr(numStart,numLen));
                    } catch (exception& e) {
                        cout << "ERROR - Unable to Load Camera Location" << endl;
                        cout << "Exception: " << e.what() << endl;
                        return 1;
                    }
                    
                    // Add value to vector
                    singleCameraLocation.push_back(thisNum);
                    
                    // Reinitialize
                    numLen = 0;
                    inNum = 0;
                } else
                    // Skip extra characters between numbers
                    if (inNum == 0) {
                        
                    } else {
                        // Must be an error
                        cout << "ERROR - Invalid character sequence in GetCameraLocations" << endl;
                        return 1;
                    }
        
        // Increment character index
        ind++;
    }
    
    // Make sure we have 3 coordinates (x,y,theta)
    if (singleCameraLocation.size() != 3) {
        // Incorrect number of coordinates
        cout << "ERROR - Incorrect number of arguments in line '" << line << "'" << endl;
        cout << "Must have 'x,y,theta;'" << endl;
        return 1;
    }
    // Return
    return 0;
}


/* Function to update simulator with details
 *  Input(s):
 *      vector<string> lines    - vector of lines of details read from file
 *      RVOSimulator* sim       - simulator object to be updated
 *  Output(s):
 *      int               - 0 if successful, 1 if failed
 *      float lengthOfSim - length of simulation time (in seconds)
 */
int processSimDetails (vector<string> lines,
                       RVO::RVOSimulator* sim,
                       float& lengthOfSim)
{
    // Read Sim Timestep
    float timestep;
    try {
        if (!getSimAgentLineDetails(lines[0],timestep)) {
            sim->setTimeStep(timestep);
        } else {
            cout << "ERROR - Unable to get timestep" << endl;
            return 1;
        }
    } catch (exception& e) {
        cout << "ERROR - Unable to Load Sim Timestep" << endl;
        cout << "Exception: " << e.what() << endl;
        return 1;
    }
    
    // Read Length of sim (in seconds)
    try {
        if (getSimAgentLineDetails(lines[1],lengthOfSim)) {
            cout << "ERROR - Unable to get length of sim" << endl;
            return 1;
        }
    } catch (exception& e) {
        cout << "ERROR - Unable to Load length of sim" << endl;
        cout << "Exception: " << e.what() << endl;
        return 1;
    }
    
    // Return
    return 0;
}


/* Function to update simulator with default agent details
 *  Input(s):
 *      vector<string> lines    - vector of lines of details read from file
 *      RVOSimulator* sim       - simulator object to be updated
 *  Output(s):
 *      int                 - 0 if successful, 1 if failed
 *      int numActiveAgents - number of agents to have moving in the simulation at a
 *                            single timestep
 */
int processAgentDetails(vector<string> lines,
                        RVO::RVOSimulator* sim,
                        int& numActiveAgents)
{
    // Read Agent Radius, Max Speed
    float radius;
    float maxSpeed;
    try {
        if ((!getSimAgentLineDetails(lines[0],radius)) &&
            (!getSimAgentLineDetails(lines[1],maxSpeed))) {
            sim->setAgentDefaults(15.0f, 10, 10.0f, 5.0f, radius, maxSpeed);
        } else {
            cout << "ERROR - Unable to get radius and max speed" << endl;
        }
    } catch (exception& e) {
        cout << "ERROR - Unable to Load Agent default params" << endl;
        cout << "Exception: " << e.what() << endl;
        return 1;
    }
    
    // Read number of agents
    float numAgents_f;
    try {
        if (getSimAgentLineDetails(lines[2],numAgents_f)) {
            cout << "ERROR - Unable to get number of agents" << endl;
        }
    } catch (exception& e) {
        cout << "ERROR - Unable to Load number of agents" << endl;
        cout << "Exception: " << e.what() << endl;
        return 1;
    }
    numActiveAgents = (int) numAgents_f;
    
    // Return
    return 0;
}



/* Function to update simulator with scene description
 *  Input(s):
 *      vector<string> obstacles    - vector of lines of coordinates read from file
 *      RVOSimulator* sim           - simulator object to be updated
 *  Output(s):
 *      int                         - 0 if successful, 1 if failed
 */
int processScene(vector<string> obstacles,
                 RVO::RVOSimulator* sim)
{
    // Process each obstacle (line)
    for (size_t i = 0; i < obstacles.size(); i++) {
        vector< vector<float> > obstacleCoordinates;
        // Convert line to floating point coordinates
        if (!getSceneDetails(obstacles[i],obstacleCoordinates)) {
            // Process each set of coordinates for this obstacle
            vector<RVO::Vector2> vertices;
            for (size_t j = 0; j < obstacleCoordinates.size(); j++) {
                // Make sure two floats in this coordinate
                if (obstacleCoordinates[j].size() == 2) {
                    RVO::Vector2 coord = RVO::Vector2(obstacleCoordinates[j][0],
                                                      obstacleCoordinates[j][1]);
                    vertices.push_back(coord);
                } else {
                    cout << "ERROR - Improper Coordinate Size: " << obstacles[i] << " - Coordinate[" << j << "]" << endl;
                    return 1;
                }
            }
            
            // Add obstacle
            sim->addObstacle(vertices);
        } else {
            cout << "ERROR - Unable to add obstacle: " << obstacles[i] << endl;
            return 1;
        }
    }
    
    // Process Obstacles so that they are accounted for in the sim
    sim->processObstacles();
    
    // Return
    return 0;
}

/* Function to get possible entrances/exits for agents
 *  Input(s):
 *      vector<string> lines    - vector of lines of coordinates read from file
 *  Output(s):
 *      int                             - 0 if successful, 1 if failed
 *      vector<vector<RVO::Vector2>> entrances - vector containing vectors describing
 *                                        entrance/exit lines by endpoints
 *                                        (i.e. [[(x0_0,y0_0),(x1_0,y1_0)],
 *                                               [(x0_1,y0_1),(x1_1,y1_1)], ...
 *                                              ])
 */
int processAgentEntrancesExits(vector<string> lines,
                               vector< vector<RVO::Vector2> >& entrances)
{
    // Process each entrance/exit
    for (size_t i = 0; i < lines.size(); i++) {
        vector< vector<float> > endpointCoordinates;
        // Convert line to floating point coordinates
        if (!getSceneDetails(lines[i],endpointCoordinates)) {
            // Process each set of coordinates for this obstacle
            vector<RVO::Vector2> vertices;
            for (size_t j = 0; j < endpointCoordinates.size(); j++) {
                // Make sure two floats in this coordinate
                if (endpointCoordinates[j].size() == 2) {
                    vertices.push_back(RVO::Vector2(endpointCoordinates[j][0],
                                                    endpointCoordinates[j][1]));
                } else {
                    cout << "ERROR - Improper Coordinate Size: " << lines[i] << " - Coordinate[" << j << "]" << endl;
                }
            }
            
            // Add this entrance/exit to the output
            entrances.push_back(vertices);
            
        }
    }
    
    // Return
    return 0;
}


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
                                 vector< vector<float> >& probs)
{
    // Process each line of entrance->exit probabilities
    for (size_t i = 0; i < lines.size(); i++) {
        vector<float> singleEntranceProbs;
        // Convert line to floating point coordinates
        if (!getEntranceProbabilities(lines[i],singleEntranceProbs)) {
            // Add these entrance->exit probablities to the output
            probs.push_back(singleEntranceProbs);
        }
    }
    
    // Return
    return 0;
}

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
                           vector< vector<float> >& cams)
{
    // Process each line of camera locations
    for (size_t i = 0; i < lines.size(); i++) {
        vector<float> singleCameraLocation;
        // Convert line to floating point coordinates
        if (!getCameraLocations(lines[i],singleCameraLocation)) {
            // Add these camera coordinates to the output
            cams.push_back(singleCameraLocation);
        }
    }
    
    // Return
    return 0;
}

/*
int main(int argc, char* argv[])
{
    if (argc < 2) {
		return 0;
	}
    
	char* filename = argv[1];
	ifstream inFile(filename);
	if (! inFile) {
		cout << "Cannot load file " << filename << endl;
	} else {
		string data;
        int i = 0;
        int section_num = -1;
        vector<string> section0;
        vector<string> section1;
        vector<string> section2;
        vector<string> section3;
        vector<string> section4;
		while (getline(inFile,data)) {
            cout << "Line " << i << ": " << data << endl;
            // Check if #
            if (data.find('#') != string::npos) {
                section_num ++;
            } else if (data.size() != 0) {
                if (section_num == 0) {
                    section0.push_back(data);
//                    float val;
//                    if (!getSimAgentLineDetails(data,val)) {
//                        cout << "No sim line details" << endl;
//                    } else {
//                        cout << "Detail " << " " << val << endl;
//                    }
                } else if (section_num == 1) {
                    section1.push_back(data);
                } else if (section_num == 2) {
                    section2.push_back(data);
//                    vector< vector<float> > verts;
//                    if (!getSceneDetails(data,verts)) {
//                        cout << "No Scene Details" << endl;
//                    } else {
//                        for (size_t j = 0; j < verts.size(); j++) {
//                            for (size_t k = 0; k < verts[j].size(); k++) {
//                                cout << verts[j][k] << " ";
//                            }
//                            cout << endl;
//                        }
//                    }
                } else if (section_num == 3) {
                    section3.push_back(data);
                } else {
                    section4.push_back(data);
//                    vector<float> probs;
//                    if (!getEntranceProbabilities(data,probs)) {
//                        cout << "No Probs" << endl;
//                    } else {
//                        for (size_t j = 0; j < probs.size(); j++) {
//                            cout << probs[j] << " ";
//                        }
//                        cout << endl;
//                    }
                }
            } else {
                cout << "skipping line" << endl;
            }
            
            i ++;
		}
        
        // Create Sim
        RVO::RVOSimulator* sim = new RVO::RVOSimulator();
        
        // Process Section 0
        cout << "\n\nProcessing Section 0" << endl;
        float lengthOfSim;
        if (processSimDetails(section0,sim,lengthOfSim)) {
            cout << "ERROR - Section 0" << endl;
        } else {
            // Print timestep and length of sim
            cout << "Section 0: timestep=" << sim->getTimeStep() << " lengthofSim=" << lengthOfSim << endl;
        }
        
        // Process Section 1
        cout << "\n\nProcessing Section 1" << endl;
        int numAgents;
        if (processAgentDetails(section1,sim,numAgents)) {
            cout << "ERROR - Section 1" << endl;
        } else {
            sim->addAgent(RVO::Vector2(1.0f,25.0f));
            // Print Default Agent Details
            cout << "Agent Details:" << endl;
            cout << "Neighbor Dist=" << sim->getAgentNeighborDist(0) << endl;
            cout << "Max Neighbors=" << sim->getAgentMaxNeighbors(0) << endl;
            cout << "Time Horiz=" << sim->getAgentTimeHorizon(0) << endl;
            cout << "Time Horiz(obst)=" << sim->getAgentTimeHorizonObst(0) << endl;
            cout << "**Radius=" << sim->getAgentRadius(0) << endl;
            cout << "**Max Speed=" << sim->getAgentMaxSpeed(0) << endl;
            // Print number of agents in sim
            cout << "**Number of Agents=" << numAgents << endl;
        }
        
        // Process Section 2
        cout << "\n\nProcessing Section 2" << endl;
        for (size_t i = 0; i < section2.size(); i++) {
            cout << section2[i] << endl;
        }
        if (processScene(section2,sim)) {
            cout << "ERROR - Section 2" << endl;
        } else {
            cout << "Num of vertices: " << sim->getNumObstacleVertices() << endl;
            for (size_t i = 0; i < sim->getNumObstacleVertices(); i++) {
                cout << sim->getObstacleVertex(i) << endl;
            }
            
        }
        
        // Process Section 3
        cout << "\n\nProcessing Section 3" << endl;
        vector< vector<RVO::Vector2> > entrances;
        if (processAgentEntrancesExits(section3,entrances)) {
            cout << "ERROR - Section 3" << endl;
        } else {
            for (size_t i = 0; i < entrances.size(); i++) {
                cout << i << endl;
                for (size_t j = 0; j < entrances[i].size(); j++) {
                    cout << entrances[i][j] << endl;
                }
            }
        }
        
        // Process Section 4
        cout << "\n\nProcessing Section 4" << endl;
        vector< vector<float> > probs;
        if (processEntranceProbabilities(section4,probs)) {
            cout << "No probabilities" << endl;
        } else {
            for (size_t i = 0; i < probs.size(); i++) {
                for (size_t j = 0; j < probs[i].size(); j++) {
                    cout << probs[i][j] << " ";
                }
                cout << endl;
            }
        }
        
        cout << "\nClosing File" << endl;
        inFile.close();
	}

	return 0;
}
*/
