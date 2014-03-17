function [solved_cam_positions, solved_cam_angles]...
               = cam_extcal_solver_SDP1_alternating_anchors(num_cameras,...
                                          camera_distance_estimates, ...
                                          camera_angle_estimates)
%CAM_EXTCAL_SOLVER_SDP1_ALTERNATING_ANCHORS Estimate camera positions using
% Semidefinate Programming technique. Then solve angles using the angle
% averaging method. Choose the best solution from this solution, or the
% reflected, and re-angle-averaged solution.

%
% The technique requires at least
% three anchors, so start with the MDS-MAP estimate, and then randomly
% attempt the SDP method using 3 randomly chosen anchors, and update
% the result if the cost measure improves. Repeat until progress stops.
%
% Technique/formulation of the SDP approach is as described in the
% paper "Second-Order Cone Programming Relaxation of Sensor Network
% Localization" by Paul Tseng. It is a slight change of notation
% from the original version in "Semidefinite Programming for Ad Hoc
% Wireless Sensor Network Localization" by Pratik Biswas and Yinyu Ye.
%
% Inputs:
%   - num_cameras - The number of cameras.
%   - camera_distances - A list of distance constraints between cameras,
%         as rows in the format [cam_num_1, cam_num_2, distance]
%         Does NOT need to be fully connected.
% Outputs:
%   - solved_cam_positions - 2d list of points, where rows are points.
%
% Uses CVX, a convex optimization package, by CVXR, to solve the SDP.
%
%Call the position-only solver.
solved_cam_positions_unreflected ...
        = cam_pos_solver_SDP1_alternating_anchors( ...
                                         num_cameras, ...
                                         camera_distance_estimates );
%Return empty if unconnected
if isempty(solved_cam_positions_unreflected)
    solved_cam_positions = [];
    solved_cam_angles = [];
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Use the angle averaging technique to solve angles.
%Also try the reflected positions. Choose the one with the lowest cost.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[solved_cam_positions, solved_cam_angles] = ...
           cam_reflection_and_angle_solver( ...
                              num_cameras, ...
                              solved_cam_positions_unreflected, ...
                              camera_angle_estimates );
