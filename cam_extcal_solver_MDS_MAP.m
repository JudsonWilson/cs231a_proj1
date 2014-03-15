function [solved_cam_positions, solved_cam_angles]...
               = cam_extcal_solver_mds_map( num_cameras, ...
                                          camera_distance_estimates, ...
                                          camera_angle_estimates)
%CAM_EXTCAL_SOLVER_MDS_MAP Estimate camera positions using MDS-MAP.
%Then solve angles using the angle averaging method. Choose the best
%solution from this solution, or the reflected, and re-angle-averaged
%solution.
%
% Inputs:
%   - num_cameras - The number of cameras.
%   - camera_distance_estimates - A list of distance constraints between
%         cameras, as rows in the format [cam_num_1, cam_num_2, distance]
%         Does NOT need to be fully connected.
%   - camera_angle_estimates - Pairwise angle estimates, in the
%       format rows of [cam_num_a, cam_num_b, angle]. This is the angle of
%       camera_a from the chord drawn from camera_a to camera_b.
% Outputs:
%   - solved_cam_positions - 2d list of points, where rows are points.
%
% Uses Judson's MDS-MAP routine.
%

%Call the position-only solver.
solved_cam_positions_unreflected = cam_pos_solver_MDS_MAP( ...
                                         num_cameras, ...
                                         camera_distance_estimates );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Use the angle averaging technique to solve angles.
%Also try the reflected positions. Choose the one with the lowest cost.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[solved_cam_positions, solved_cam_angles] = ...
           cam_reflection_and_angle_solver( ...
                              num_cameras, ...
                              solved_cam_positions_unreflected, ...
                              camera_angle_estimates );
