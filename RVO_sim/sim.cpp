
#include <iostream>
#include <fstream>
#include <random>
#include <cmath>

#include "RVO.h"
#include "parser.hpp"

#define _VERBOSE_ 0

// 2D Vector of Primary/Secondary goals for agents
// Primary Goals get agents into middle of map (avoids getting stuck)
// Secondary Goals get agents to exits
// goals = [[goal_primary_a0, goal_secondary_a0],
//          [goal_primary_a1, goal_secondary_a1], ...
//         ]
vector< vector<RVO::Vector2> > goals;

// 1D Vectors of Boolean values stating whether each agent has reached its primary
// and secondary goal, respectively
vector<bool> reachedPrimaryGoal;
vector<bool> reachedSecondaryGoal;

// Store Tracklets in ground truth - Format: [camera number,track number, length of valid data, point0 ...]
//  [[0,trk#,numPoints0,x(t00),y(t00),t00,x(t10),y(t10),t10, ... , x(tn0),y(tn0),tn0],
//   [0,trk#,numPoints1,x(t01),y(t01),t01,x(t11),y(t11),t11, ... , x(tn1),y(tn1),tn1],
//   ... ,
//   [0,trk#,numPointsm,x(t0m),y(t0m),t0m,x(t1m),y(t1m),t1m, ... , x(tnm),y(tnm),tnm]]
vector< vector<float> > groundTruthTracks;

// Store Tracklets by camera
// [[[cam#,0,0,0],[x0,y0,t0,tr#],[x1,y1,t1,tr#], ... ],
//  [[cam#,0,0,0],[x0,y0,t0,tr#],[x1,y1,t1,tr#], ... ],
//  ... ,
//  [[cam#,0,0,0],[x0,y0,t0,tr#],[x1,y1,t1,tr#], ... ]]
vector< vector< vector<float> > > cameraTracks;

// Store Tracklet IDs by camera
// [[id0cam0,id1cam0,...,idMcam0],
//  [id0cam1,id1cam1,...,idKcam1],
//  ...,
//  [id0camN,id1camN,...,idLcamN]]
vector< vector<float> > cameraUniqueTracks;

// Create Random Number Gen
default_random_engine generator;
uniform_real_distribution<float> distribution(0.0,1.0);


// Read scene and agent details from input file
int parseFile(char* filename,
              vector< vector<string> >& sections);

// Set up scene in the simulator
int setupScenario(RVO::RVOSimulator* sim,
                  vector< vector<string> > sections,
                  float& lengthOfSim,
                  int& numAgents,
                  vector< vector<RVO::Vector2> >& entrances,
                  vector< vector<float> >& transitionProbs,
                  vector< vector<float> >& cameraLocations);

// Set Agent's Start and End Points
int getEntranceIndex(int numEntrances);
int getExitIndex(vector<float> transitionProb);
RVO::Vector2 getEntranceExitLocation(RVO::Vector2 loc1,
                                     RVO::Vector2 loc2);

// Create New Agents
void updateAgents(RVO::RVOSimulator* sim,
                  int maxNumActiveAgents,
                  int& numActiveAgents,
                  vector< vector<RVO::Vector2> > entrances,
                  vector< vector<float> > transitionProbs);

// Capture agent movements
void updateVisualization(RVO::RVOSimulator* sim);
bool reachedGoal(RVO::RVOSimulator* sim);
void setPreferredVelocities(RVO::RVOSimulator* sim,
                            int& numActiveAgents);

// Get tracklets in camera coordinates
int inCameraFOV(int camNum,
                vector<float> camLocation,
                vector<RVO::Vector2> camPolygon,
                int trackNum);
int getCameraTracklets(vector< vector<float> > cameraLocations);

// Write Ground Truth and Camera Coordinate Tracklets to file
int writeTracks(char* outFilename);


// Main Function
int main(int argc, char* argv[])
{
    // Load Input Filename
    char* filename;
    if (argc < 2) {
        cout << "Loading environment from file.txt" << endl;
        filename = "example.txt";
    } else {
        filename = argv[1];
        cout << "Loading environment from " << filename << endl;
    }
    
    // Create a new Simulator Instance
    RVO::RVOSimulator* sim = new RVO::RVOSimulator();
    
    // Parse File
    vector< vector<string> > sections;
    if (parseFile(filename, sections)) {
        cout << "Unable to load file " << filename << endl;
        return 1;
    }
    
    // Set up scenario
    float lengthOfSim;
    int numActiveAgents = 0;
    int maxNumActiveAgents;
    vector< vector<RVO::Vector2> > entrances;
    vector< vector<float> > transitionProbs;
    vector< vector<float> > cameraLocations;
    if(setupScenario(sim,
                     sections,
                     lengthOfSim,
                     maxNumActiveAgents,
                     entrances,
                     transitionProbs,
                     cameraLocations)) {
        cout << "Unable to setup scenario" << endl;
        return 1;
    }
    
    // Simulate
    do {
        // Add Agents (if necessary)
        if (_VERBOSE_)
            cout << "Update Agents" << endl;
        updateAgents(sim,
                     maxNumActiveAgents,
                     numActiveAgents,
                     entrances,
                     transitionProbs);
        // Record Agent Locations
        if (_VERBOSE_)
            cout << "Update Visualization" << endl;
        updateVisualization(sim);
        // Set Agent Preferred Velocities
        if (_VERBOSE_)
            cout << "Update Velocities" << endl;
        setPreferredVelocities(sim,numActiveAgents);
        // Simulate
        if (_VERBOSE_)
            cout << "Simulate" << endl;
        sim->doStep();
        if (_VERBOSE_)
            cout << endl;
    } while (!(sim->getGlobalTime() > lengthOfSim)); //(reachedGoal(sim) ||
    
    // Put into Camera Coordinates
    int tmp = getCameraTracklets(cameraLocations);
     
    // Load Output Filename
    char* outFilename;
    if (argc < 3) {
        cout << "Writing to out.csv" << endl;
        outFilename = "out";
    } else {
        outFilename = argv[2];
        cout << "Writing to " << outFilename << endl;
    }
     
    // Write Tracks to Files
    tmp = writeTracks(outFilename);
    
    // Track Stats
    cout << "Total Num Tracks: " << groundTruthTracks.size() << endl;
    cout << "Num Tracks by cam: ";
    for (size_t i = 0; i < cameraUniqueTracks.size(); i++) {
        cout << cameraUniqueTracks[i].size() << " ";
    }
    cout << endl;
    
    // Close Scenario
    delete sim;
    
    return 0;
}


int parseFile(char* filename,
              vector< vector<string> >& sections)
{
    // Open File
    ifstream inFile(filename);
    if (! inFile) {
        cout << "Unable to load file " << filename << endl;
        return 1;
    }
    
    string data;
    int i = 0;
    int section_num = -1;
    vector<string> section;
    
    // Read File Line by Line
    while (getline(inFile,data)) {
        // Check if #
        if (data.find('#') != string::npos) {
            // Store previous section to sections
            if (section_num >= 0) {
                sections.push_back(section);
                // Reset section
                section.clear();
            }
            section_num ++;
        } else if (data.size() != 0) {
            section.push_back(data);
        }
        i++;
    }
    // Store last section
    sections.push_back(section);
    
    // Close File
    inFile.close();
    
    // Return
    return 0;
}


int setupScenario(RVO::RVOSimulator* sim,
                  vector< vector<string> > sections,
                  float& lengthOfSim,
                  int& numActiveAgents,
                  vector< vector<RVO::Vector2> >& entrances,
                  vector< vector<float> >& transitionProbs,
                  vector< vector<float> >& cameraLocations)
{
    // Check that we have 5 sections
    if (sections.size() != 6) {
        cout << "Need 6 sections, have " << sections.size() << endl;
        return 1;
    }
    
    // Process Section 0 - Sim Details
    if (processSimDetails(sections[0],sim,lengthOfSim)) {
        cout << "ERROR - Unable to process Section 0" << endl;
        return 1;
    } else {
        /*
        // Print timestep and length of sim
        cout << "Section 0: timestep=" << sim->getTimeStep() << " lengthofSim=" << lengthOfSim << endl;
         */
    }
    
    // Process Section 1 - Agent Details
    if (processAgentDetails(sections[1],sim,numActiveAgents)) {
        cout << "ERROR - Unable to process Section 1" << endl;
        return 1;
    } else {
        /*
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
         */
    }
    
    // Process Section 2 - Scene Details
    if (processScene(sections[2],sim)) {
        cout << "ERROR - Unable to process Section 2" << endl;
        return 1;
    } else {
        /*
        cout << "Num of vertices: " << sim->getNumObstacleVertices() << endl;
        for (size_t i = 0; i < sim->getNumObstacleVertices(); i++) {
            cout << sim->getObstacleVertex(i) << endl;
        }
         */
        
    }
    
    // Process Section 3 - Entrance/Exits
    if (processAgentEntrancesExits(sections[3],entrances)) {
        cout << "ERROR - Unable to process Section 3" << endl;
        return 1;
    } else {
        /*
        for (size_t i = 0; i < entrances.size(); i++) {
            cout << i << endl;
            for (size_t j = 0; j < entrances[i].size(); j++) {
                cout << entrances[i][j] << endl;
            }
        }
         */
    }
    
    // Process Section 4 - Agent Transition Probabilities
    if (processEntranceProbabilities(sections[4],transitionProbs)) {
        cout << "ERROR - Unable to process Section 4" << endl;
        return 1;
    } else {
        /*
        for (size_t i = 0; i < transitionProbs.size(); i++) {
            for (size_t j = 0; j < transitionProbs[i].size(); j++) {
                cout << transitionProbs[i][j] << " ";
            }
            cout << endl;
        }
         */
    }
    
    // Process Section 5 - Camera Locations
    if (processCameraLocations(sections[5],cameraLocations)) {
        cout << "ERROR - Unable to process Section 5" << endl;
        return 1;
    } else {
        /*
        for (size_t i = 0; i < cameraLocations.size(); i++) {
            for (size_t j = 0; j < cameraLocations[i].size(); j++) {
                cout << cameraLocations[i][j] << " ";
            }
            cout << endl;
        }
         */
    }
    
    // Return
    return 0;
}


int getEntranceIndex(int numEntrances)
{
    // Generate Random Number
    float randomNum = distribution(generator);
    
    // Compute Entrance Index
    int entranceIndex = (int) (numEntrances*randomNum + 0.5);
    
    // Return
    return entranceIndex;
}


int getExitIndex(vector<float> transitionProb)
{
//    // Create Random Number Gen
//    default_random_engine generator;
//    uniform_real_distribution<float> distribution(0.0,1.0);
    
    // Generate Random Number
    float randomNum = distribution(generator);
    
    // Compute Exit Index
    float sum = 0;
    int exitIndex = 0;
    for (size_t i = 0; i < transitionProb.size(); i ++) {
        sum += transitionProb[i];
        if (sum >= randomNum) {
            exitIndex = i;
            break;
        }
    }
    
    // Return
    return exitIndex;
}


RVO::Vector2 getEntranceExitLocation(RVO::Vector2 loc1,
                                     RVO::Vector2 loc2)
{
//    // Create Random Number Gen
//    default_random_engine generator;
//    uniform_real_distribution<float> distribution(0.0,1.0);
    
    // Generate Random Number
    float percentage = distribution(generator);
    
    // Compute Output Location
    RVO::Vector2 outLoc = loc1 + percentage*(loc2-loc1);
    
    // Return
    return outLoc;
}


void updateAgents(RVO::RVOSimulator* sim,
                  int maxNumActiveAgents,
                  int& numActiveAgents,
                  vector< vector<RVO::Vector2> > entrances,
                  vector< vector<float> > transitionProbs)
{
    // Add Agents until we have the maximum number of active agents
    if (_VERBOSE_)
        cout << "Active: " << numActiveAgents << ", Max: " << maxNumActiveAgents << endl;
    while (numActiveAgents < maxNumActiveAgents) {
        // Get Entrance Index
        if (_VERBOSE_)
            cout << "Entrance Ind" << endl;
        int entranceIndex = getEntranceIndex(transitionProbs.size()-1);
        
        // Get Entrance Location
        if (_VERBOSE_)
            cout << "Entrance Loc - Ind: " << entranceIndex << " of " << transitionProbs.size() << endl;
        RVO::Vector2 entranceLoc = getEntranceExitLocation(entrances[entranceIndex][0],
                                                           entrances[entranceIndex][1]);
        
        // Get Exit Index
        if (_VERBOSE_)
            cout << "Exit Ind" << endl;
        int exitIndex = getExitIndex(transitionProbs[entranceIndex]);
        
        // Get Exit Locations
        if (_VERBOSE_)
            cout << "Exit Locs" << endl;
        RVO::Vector2 primaryExitLoc = getEntranceExitLocation(entrances[exitIndex][2],
                                                              entrances[exitIndex][3]);
        RVO::Vector2 secondaryExitLoc = getEntranceExitLocation(entrances[exitIndex][4],
                                                                entrances[exitIndex][5]);
        
        // Add Agent
        if (_VERBOSE_)
            cout << "Add Agent" << endl;
        sim->addAgent(entranceLoc);
        if (_VERBOSE_)
            cout << "Agent Added" << endl;
        // Add Agent's primary and secondary goals
        vector<RVO::Vector2> exits;
        exits.push_back(primaryExitLoc);
        exits.push_back(secondaryExitLoc);
        goals.push_back(exits);
        // Initialize Agent's reachedPrimaryGoal, reachedSecondaryGoal
        reachedPrimaryGoal.push_back(false);
        reachedSecondaryGoal.push_back(false);
        // Initialize Agent's track with camera index (0 for now),
        // track index, and length of valid data (0 for now)
        vector<float> track;
        track.push_back(0);
        track.push_back(groundTruthTracks.size());
        track.push_back(0);
        groundTruthTracks.push_back(track);
        
        // Update Number of Active Agents
        numActiveAgents += 1;
    }
}


void updateVisualization(RVO::RVOSimulator* sim)
{
    // Output the current global time
    if (_VERBOSE_)
        std::cout << sim->getGlobalTime() << " ";
    
    // Output the position for all the agents
    for (size_t i = 0; i < sim->getNumAgents(); i++) {
        RVO::Vector2 loc = sim->getAgentPosition(i);
        if (_VERBOSE_)
            std::cout << loc << " ";
        // Store Position and time if agent isn't at final goal
        if (!reachedSecondaryGoal[i]) {
            groundTruthTracks[i].push_back(loc.x());
            groundTruthTracks[i].push_back(loc.y());
            groundTruthTracks[i].push_back(sim->getGlobalTime());
            // Increment the length of valid data
            groundTruthTracks[i][2] += 1;
        }
    }
    
    if (_VERBOSE_)
        cout << endl;
    
    
    if (_VERBOSE_)
        cout << "Goals: ";
    for (size_t i = 0; i < sim->getNumAgents(); i++) {
        if (reachedPrimaryGoal[i]) {
            if (reachedSecondaryGoal[i]) {
                if (_VERBOSE_)
                    cout << "**";
            } else {
                if (_VERBOSE_)
                    cout << "*";
            }
            if (_VERBOSE_)
                cout << goals[i][1] << " ";
        } else {
            if (_VERBOSE_)
                cout << goals[i][0] << " ";
        }
    }
    if (_VERBOSE_)
        cout << endl;
}


bool reachedGoal(RVO::RVOSimulator* sim)
{
    // Check whether all agents have arrived at their goals
    for (size_t i = 0; i < sim->getNumAgents(); i++) {
        if (!reachedSecondaryGoal[i]) {
            // Agent i is further away from his secondary goal than one radius
            return false;
        }
    }
    // All agents within radius of their goals
    return true;
}


void setPreferredVelocities(RVO::RVOSimulator* sim,
                            int& numActiveAgents)
{
    // Set the preferred velocity for each agent
    for (size_t i = 0; i < sim->getNumAgents(); i++) {
        if (reachedPrimaryGoal[i]) {
            // Agent has passed primary goal -> Headed to secondary goal
            if (absSq(goals[i][1] - sim->getAgentPosition(i)) < (sim->getTimeStep() * sim->getAgentMaxSpeed(i)) * (sim->getTimeStep() * sim->getAgentMaxSpeed(i))) {
                //Agent is within (max speed * timestep) of its secondary goal, set preferred velocity to 0
                sim->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
                // First time at goal, reduce number of active agents, and set reachedSecondaryGoal to TRUE
                if (!reachedSecondaryGoal[i]) {
                    numActiveAgents -= 1;
                    reachedSecondaryGoal[i] = true;
                }
            } else {
                // Agent is far away from its goal, set preferred velocity as unit vector times max speed towards agent's goal
                sim->setAgentPrefVelocity(i, sim->getAgentMaxSpeed(i)*normalize(goals[i][1] - sim->getAgentPosition(i)));
            }
        } else {
            // Agent has not passed primary goal -> Headed to primary goal
            if (absSq(goals[i][0] - sim->getAgentPosition(i)) < (sim->getTimeStep() * sim->getAgentMaxSpeed(i)) * (sim->getTimeStep() * sim->getAgentMaxSpeed(i))) {
                //Agent is within (max speed * timestep) of its primary goal, head to secondary goal
                reachedPrimaryGoal[i] = true;
                sim->setAgentPrefVelocity(i, sim->getAgentMaxSpeed(i)*normalize(goals[i][1] - sim->getAgentPosition(i)));
            } else {
                // Agent is far away from its goal, set preferred velocity as unit vector times max speed towards agent's goal
                sim->setAgentPrefVelocity(i, sim->getAgentMaxSpeed(i)*normalize(goals[i][0] - sim->getAgentPosition(i)));
            }
        }
    }
}


int inCameraFOV(int camNum,
                vector<float> camLocation,
                vector<RVO::Vector2> camPolygon,
                int trackNum)
{
    // Get Camera Location
    float x0 = camLocation[0];
    float y0 = camLocation[1];
    RVO::Vector2 camCenter = RVO::Vector2(x0,y0);
    float theta = M_PI*camLocation[2]/180;
    vector<RVO::Vector2> inv_rotation;
    inv_rotation.push_back(RVO::Vector2(cos(-theta),-sin(-theta)));
    inv_rotation.push_back(RVO::Vector2(sin(-theta),cos(-theta)));
    
    // Capture Track Information
    float trackID = groundTruthTracks[trackNum][1];
    
    // Keep track of whether tracklet is in this camera
    size_t trackletInCam = 0;
    
    // Find all points in this track which fall in this camera's FOV
    for (size_t i = 0; i < groundTruthTracks[trackNum].size()/3-1; i++) {
        // Get Point
        float x = groundTruthTracks[trackNum][3*(i+1)];
        float y = groundTruthTracks[trackNum][3*(i+1) + 1];
        RVO::Vector2 p = RVO::Vector2(x,y);
        float t = groundTruthTracks[trackNum][3*(i+1) + 2];
        
        // Check if Point is within polygon
        size_t validSignFlag = 1;
        float prevResult= 0;
        for (size_t j = 0; j < camPolygon.size(); j++) {
            // Get Vertices defining line segment
            RVO::Vector2 A = camPolygon[j];
            RVO::Vector2 B;
            if (j+1 == camPolygon.size()) {
                B = camPolygon[0];
            } else {
                B = camPolygon[j+1];
            }
            RVO::Vector2 perp = RVO::Vector2(-(B.y()-A.y()),(B.x()-A.x()));
            
            // Check Sign
            float result = perp*(p-A);
            // First Run through => capture result
            if (prevResult == 0) {
                prevResult = result;
                // Move to next segment if sign is same
            } else if (prevResult*result > 0) {
                prevResult = result;
                // Skip if point is on this line
            } else if (result == 0) {
                continue;
                // Sign is different => Point not in polygon
            } else {
                validSignFlag = 0;
                break;
            }
        }
        
        // Capture Point if in polygon
        if (validSignFlag) {
            // Update status of the tracklet occuring in this camera
            trackletInCam = 1;
            
            // Convert groundTrack point to camera frame
            RVO::Vector2 p_cam = RVO::Vector2(inv_rotation[0]*(p-camCenter),
                                              inv_rotation[1]*(p-camCenter));
            // Add Point
            vector<float> point;
            point.push_back(p_cam.x());
            point.push_back(p_cam.y());
            point.push_back(t);
            point.push_back(trackID);
            // Add to cameraTracks
            cameraTracks[camNum].push_back(point);
        }
    }
    
    // Add tracklet ID to cameraUniqueTracks
    if (trackletInCam) {
        cameraUniqueTracks[camNum].push_back(trackNum);
    }
    
    // Return
    return 0;
}


int getCameraTracklets(vector< vector<float> > cameraLocations)
{
    // Camera Field of View (centered at x=0,y=0,theta=0)
    vector<RVO::Vector2> camPolygon;
    camPolygon.push_back(RVO::Vector2(9.0f,1.0f));
    camPolygon.push_back(RVO::Vector2(17.5f,5.5f));
    camPolygon.push_back(RVO::Vector2(17.5f,-5.5f));
    camPolygon.push_back(RVO::Vector2(9.0f,-1.0f));
    
    // Find tracklets contained in each camera's field of view
    for (size_t i = 0; i < cameraLocations.size(); i++) {
        // Initialize Camera in cameraTracks, cameraUniqueTracks
        vector<float> firstRow;
        firstRow.push_back(i);  // Camera Number
        vector< vector<float> > cameraTrack;
        cameraTrack.push_back(firstRow);
        cameraTracks.push_back(cameraTrack);
        vector<float> trackIDsInCam;
        cameraUniqueTracks.push_back(trackIDsInCam);
        
        // Determine Camera's Vertices
        float x0 = cameraLocations[i][0];
        float y0 = cameraLocations[i][1];
        RVO::Vector2 camCenter = RVO::Vector2(x0,y0);
        float theta = M_PI*cameraLocations[i][2]/180;
        vector<RVO::Vector2> rotation;
        rotation.push_back(RVO::Vector2(cos(theta),-sin(theta)));
        rotation.push_back(RVO::Vector2(sin(theta),cos(theta)));
        vector<RVO::Vector2> camPolygonRot;
        for (size_t j = 0; j < camPolygon.size(); j++) {
            camPolygonRot.push_back(RVO::Vector2(rotation[0]*camPolygon[j],
                                                 rotation[1]*camPolygon[j]) +
                                    camCenter);
            //            cout << camPolygonRot[j].x() << " " << camPolygonRot[j].y() << endl;
        }
        
        // Loop through all tracks
        for (size_t j = 0; j < groundTruthTracks.size(); j++) {
            // Add all points in this track that are in this camera's FOV to cameraTracks
            if (inCameraFOV(i,cameraLocations[i],camPolygonRot,j)) {
                cout << "ERROR - Unable to determine camera tracklets for camera: " << i << ", track: " << j << endl;
            }
            
        }
        
    }
    
    // Return
    return 0;
}


int writeTracks(char* outFilename)
{
    // Find Max Vector Length
    size_t maxVectorLen = 0;
    for (size_t i = 0; i < groundTruthTracks.size(); i++) {
        if (groundTruthTracks[i][2] > maxVectorLen) {
            maxVectorLen = groundTruthTracks[i][2];
        }
    }
    // Extend all vectors to maxVectorLen (pad with -1)
    for (size_t i = 0; i < groundTruthTracks.size(); i++) {
        for (size_t j = groundTruthTracks[i].size() - 3; j < 3*maxVectorLen; j++) {
            groundTruthTracks[i].push_back(-1);
        }
    }
    
    // *** Write to file ***
    // Write ground truth to <filename>.csv
    ofstream groundOutFile;
    string groundOutFilenameString;
    groundOutFilenameString.append("./data/");
    groundOutFilenameString.append(outFilename);
    groundOutFilenameString.append(".csv");
    // Open file
    groundOutFile.open(groundOutFilenameString,ios::trunc);
    if (groundOutFile.is_open()) {
        // Write Tracks to file
        for (size_t trackInd = 0; trackInd < groundTruthTracks.size(); trackInd++) {
            for (size_t j = 0; j < groundTruthTracks[trackInd].size(); j++) {
                groundOutFile << groundTruthTracks[trackInd][j] << " ";
            }
            groundOutFile << "\n";
        }
    } else {
        cout << "ERROR - Unable to write ground truth to " << groundOutFilenameString << endl;
    }
    // Close file
    groundOutFile.close();
    
    // *** Write Camera files to <filename>_cam<cam_id>.csv ***
    for (size_t i = 0; i < cameraTracks.size(); i++) {
        vector< vector<float> > cameraTrack = cameraTracks[i];
        // Open File
        ofstream camOutFile;
        string camOutFilenameString;
        camOutFilenameString.append("./data/");
        camOutFilenameString.append(outFilename);
        camOutFilenameString.append("_cam");
        camOutFilenameString.append(to_string((int) cameraTrack[0][0]));
        camOutFilenameString.append(".csv");
        // Open File
        camOutFile.open(camOutFilenameString,ios::trunc);
        if (camOutFile.is_open()) {
            // Write each point to its own line
            for (size_t pointInd = 1; pointInd < cameraTrack.size(); pointInd++) {
                camOutFile << cameraTrack[pointInd][0] << " "
                << cameraTrack[pointInd][1] << " "
                << cameraTrack[pointInd][2] << " "
                << cameraTrack[pointInd][3] << "\n";
            }
        } else {
            cout << "ERROR - Unable to write camera tracks to " << camOutFilenameString << endl;
        }
        
        // Close File
        camOutFile.close();
        
    }
    
    // Return
    return 0;
}

