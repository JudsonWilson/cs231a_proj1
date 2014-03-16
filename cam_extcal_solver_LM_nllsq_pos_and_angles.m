function [solved_cam_positions, solved_cam_angles]...
               = cam_extcal_solver_LM_nllsq_pos_and_angles(...
                                          num_cameras, ...
                                          camera_distance_estimates, ...
                                          camera_angle_estimates)
%CAM_EXTCAL_SOLVER_LM_NLLSQ_POS_AND_ANGLES Estimate camera positions
%and angles simultaneously using LM Non-Linear Least Squares,
%aka Levenberg–Marquardt algorithm (LM).
%
%Calls a 3rd-party solver: "LMFnlsq - Solution of nonlinear least squares"
%by Miroslav Balda available MathWorks MATLAB Central website.
%It calculates the Levenberg-Marquardt algorithm as modified by Fletcher.
%Initial starting position calculated using our MDS-MAP implementation.
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
%   - camera_angle_estimates - Pairwise angle estimates, in the
%       format rows of [cam_num_a, cam_num_b, angle]. This is the angle of
%       camera_a from the chord drawn from camera_a to camera_b.
% Outputs:
%   - solved_cam_positions - 2d list of points, where rows are points.
%

%Make these variables global so we can create a cost function that uses
% them for LMFnlsq
% - LMFnlsq won't pass params to functions as far as I can tell, so
%   have to use this lame hack
global glob_camera_distance_estimates;
global glob_camera_angle_estimates;

glob_camera_distance_estimates = camera_distance_estimates;
glob_camera_angle_estimates = camera_angle_estimates;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Seed with MDS-MAP result, and the reflection of it
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
start_locations_unreflected = cam_pos_solver_MDS_MAP(num_cameras, ...
                                           camera_distance_estimates );

start_locations_reflected = start_locations_unreflected;
start_locations_reflected(:,1) = -start_locations_reflected(:,1);


MDS_MAP_position_cost= calculate_camera_positions_cost(camera_distance_estimates,...
                                                start_locations_unreflected)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calculate the LM result.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start_locations = {start_locations_unreflected, start_locations_reflected};

label_strings = {'unreflected', 'reflected'};

%This will hold the cost of each of the unreflected/reflected solutions
% and we will choose the one corresponding to the smallest cost.
summed_cost = [inf inf];

%Loop through the 2 solutions
for s=1:2
    %Stack the positions into a giant stacked vector.
    start_locations_stackedvector = start_locations{s}';
    start_locations_stackedvector = start_locations_stackedvector(:);

    %starting angles are made using the angle averaging method
    start_angles = cam_angle_from_pos_solver( ...
                                      num_cameras, ...
                                      start_locations{s}, ...
                                      camera_angle_estimates );


    %Start with MDS unreflected or reflected coordinates, and camera
    %angles from averaging method
    starting_solution = [start_locations_stackedvector; ...
                         start_angles(:)];

%use this call if your matlab doesn have lsqnonlin.
%lsqnonlin seems to work a lot better
%    [C,ssq,cnt] = LMFnlsq('LM_nllsq_cost_dist_and_angles', ...
%                           starting_solution,'Display',1);

    C = lsqnonlin(@LM_nllsq_cost_dist_and_angles, starting_solution);

    %C is in the stacked vector format, turn it into a vertical list of row
    % vectors
    solved_cam_positions_unreflected = reshape(C(1:(num_cameras*2),1),...
                                               2, num_cameras)';
    solved_cam_angles_unreflected = C((num_cameras*2+1):end);

    LMFnlsq_cost= calculate_camera_positions_cost(camera_distance_estimates,...
                                              solved_cam_positions_unreflected)

    solved_cam_positions{s} = solved_cam_positions_unreflected;
    solved_cam_angles{s} = solved_cam_angles_unreflected;

    %fprintf('Using set %s\n', label_strings{s});
    pos_cost = calculate_camera_positions_cost(camera_distance_estimates,...
                                                solved_cam_positions{s});
    angle_cost = calculate_camera_angles_cost(...
                                   camera_angle_estimates, ...
                                   solved_cam_angles{s}, ...
                                   solved_cam_positions{s})
    summed_cost(s) = pos_cost + angle_cost;
end

[~, use_set] = min(summed_cost);
solved_cam_positions  = solved_cam_positions{use_set};
solved_cam_angles     = solved_cam_angles{use_set};


end
