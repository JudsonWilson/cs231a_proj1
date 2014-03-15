function [ estimated_locations_out, estimated_angles ] ...
    = cam_reflection_and_angle_solver( num_cameras,...
                                       estimated_locations,...
                                       pairwise_camera_angle_estimates )
%CAM_REFLECTION_AND_ANGLE_SOLVER Solves camera angles from estimated positions
% and angle estimates. Also attempts to reflect the points and solve
% angles, and determine which solution is best based on cost functions.
% This is done because position-only solvers will return position up
% to a reflection, but the angle constraints, ideally, lead to the correct
% solution.
%
% Inputs:
%   num_cameras - Number of cameras.
%   estimated_locations - n x 2 list of camera location estimates. These
%       positions will be used in conjunction with pairwise angles to
%       come up with the estimate.
%   pairwise_camera_angle_estimates - Pairwise angle estimates, in the
%       format rows of [cam_num_1, cam_num_2, angle]. This is the angle of
%       the camera from the chord drawn from cammera 1 to camera 2.
%
% Ouptuts:
%   estimated_locations_out - Returns either the same coordinates as
%       input, or a reflected version.
%   estimated_angles - Angles of each camera in world coordinates, as a
%       simple 1D array, or list.

estimated_locations_unreflected = estimated_locations;

%
% Calculate Reflection of Best Position - Reflext about x axis.
%
estimated_locations_reflected = estimated_locations_unreflected;
estimated_locations_reflected(:,1) = -estimated_locations_reflected(:,1);

%
% Calculate Angles - Using both the original and reflected positions.
%
estimated_angles_unreflected = cam_angle_from_pos_solver( ...
                                      num_cameras, ...
                                      estimated_locations_unreflected, ...
                                      pairwise_camera_angle_estimates );
estimated_angles_reflected = cam_angle_from_pos_solver( ...
                                      num_cameras, ...
                                      estimated_locations_reflected, ...
                                      pairwise_camera_angle_estimates );

%
%Compute the angle cost function for both cases
%
cost_unreflected = calculate_camera_angles_cost(...
                                   pairwise_camera_angle_estimates, ...
                                   estimated_angles_unreflected, ...
                                   estimated_locations_unreflected);
cost_reflected = calculate_camera_angles_cost(...
                                   pairwise_camera_angle_estimates, ...
                                   estimated_angles_reflected, ...
                                   estimated_locations_reflected);

%
% Keep the results with the best (lowest) cost function.
%
if cost_reflected < cost_unreflected
    %fprintf('Keeping reflected\n');
    estimated_locations_out = estimated_locations_reflected;
    estimated_angles = estimated_angles_reflected;
else
    %fprintf('Keeping unreflected\n');
    estimated_locations_out = estimated_locations_unreflected;
    estimated_angles = estimated_angles_unreflected;
end

end