function [ solved_cam_positions ]  ...
                      = cam_pos_solver_SDP1_alternating_anchors ...
                            (num_cameras, camera_distance_estimates )
%CAM_POS_SOLVER_SDP1_ALTERNATING_ANCHORS Estimate camera positions using
% Semidefinate Programming technique. The technique requires at least
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

if num_cameras < 4
    error(['The SDP technique only makes sense for more than 3 cameras,'...
           'as it requires 3 anchors.']);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First use MDS-MAP to make first-pass estimate, store as our best 
% (current) result.
best_locations = cam_pos_solver_MDS_MAP(num_cameras, ...
                                        camera_distance_estimates);
best_cost = calculate_camera_positions_cost(camera_distance_estimates, ...
                                             best_locations); 

%fprintf('MDS-MAP cost: %f \n', best_cost);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Choose 3 random cameras as anchors, try algorithm, update the result
% if it improves the cost measure. Halt after 20 iterations of no
% improvement.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runs_without_improvement = 0;
while runs_without_improvement < 20
    %Pick 3 random anchors
    anchors = sort(randsample(num_cameras, 3), 'ascend');

    new_locations = cam_pos_solver_SDP1_3plusanchors(num_cameras, ...
                                      camera_distance_estimates, ...
                                      anchors, best_locations(anchors,:));
    new_cost = calculate_camera_positions_cost(camera_distance_estimates, new_locations); 

    if ~isreal(new_locations)
        disp(new_locations);
        error('unreal cam_pos_solver_SDP1_3plusanchors result!');
    end
    
%    fprintf('Resulting SDP1 cost: %f ', new_cost);
    if new_cost < best_cost
%        fprintf('Keeping new results!\n');
        best_locations = new_locations;
        best_cost = new_cost;
        runs_without_improvement = 0;
    else
%        fprintf('Rejecting new results!\n');
        runs_without_improvement = runs_without_improvement + 1;
    end
end

%Return the best result.
solved_cam_positions = best_locations;
end

