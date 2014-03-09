function [ solved_cam_positions ] = cam_pos_solver_LM_nllsq ...
                                        (num_cameras, ...
                                         camera_distance_estimates )
%CAM_POS_SOLVER_LM_NLLSQ Estimate camera positions using LM Non-Linear
%Least Squares, aka Levenberg�Marquardt algorithm (LMA). Calls a 3rd-party
%solver: "LMFnlsq - Solution of nonlinear least squares" by Miroslav Balda
%available MathWorks MATLAB Central website. It calculates the 
%Levenberg-Marquardt algorithm as modified by Fletcher. Initial starting
%position calculated using our MDS-MAP implementation.
%
%The use of this technique for sensor localization is described by
%Ellis and Hazas in "A comparison of MDS-MAP and non-linear regression."
%They claim Shang and Ruml use it in "Improved MDS-based Localization" as a
%refinement step on subsets of the network, which are each calculated
%using local neighbors, and then are merged using other techniques.
%
% Inputs:
%   - num_cameras - The number of cameras.
%   - camera_distances - A list of distance constraints between cameras,
%         as rows in the format [cam_num_1, cam_num_2, distance]
%         Does NOT need to be fully connected.
% Outputs:
%   - solved_cam_positions - 2d list of points, where rows are points.
%

%Make these variables global so we can create a cost function that uses
% them for LMFnlsq
% - LMFnlsq won't pass params to functions as far as I can tell, so
%   have to use this lame hack
global glob_camera_distance_estimates;

glob_camera_distance_estimates = camera_distance_estimates;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Seed with MDS Map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MDS_MAP_locations = cam_pos_solver_MDS_MAP(num_cameras, ...
                                           camera_distance_estimates );
MDS_MAP_cost= calculate_camera_positions_cost(camera_distance_estimates,...
                                                MDS_MAP_locations)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calculate the LM result.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                            
%Stack the positions into a giant stacked vector.
MDS_MAP_locations_stackedvector = MDS_MAP_locations';
MDS_MAP_locations_stackedvector = MDS_MAP_locations_stackedvector(:);

[C,ssq,cnt] = LMFnlsq('LM_nllsq_cost', MDS_MAP_locations_stackedvector);

%C is in the stacked vector format, turn it into a vertical list of row
% vectors
solved_cam_positions = reshape(C,2,length(C)/2)';

LMFnlsq_cost= calculate_camera_positions_cost(camera_distance_estimates,...
                                                solved_cam_positions)


end