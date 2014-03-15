function [estimated_cameras, camera_relation_votes_and_centers] ...
                         = solve_cameras_extcal(correspondences, algorithm)
%SOLVE_CAMERAS_EXTCAL Takes in a bunch of correspondences, and using the
%chosen algorithm, estimates the external calibration of all the cameras up
%to a rotation/translation.
%
%  Inputs:
%      correspondences - A struct with all the captured correspondences.
%          Structure format:
%               .num_cameras - The number of cameras.
%               .tracklets_cam_coords(i).cam_num - Number of the camera
%                                                  that the tracklet is in.
%               .tracklets_cam_coords(i).path    - (n x 2) list of
%                                                  coordinates
%               .tracklets_cam_coords(i).first_time - time of first point.
%                                                  This time scheme may
%                                                  need to change in the
%                                                  future. Right now you
%                                                  just assume each point
%                                                  is a time step.
%               .tracklet_pairings - An n x 2 list of pairings,
%                                    identifying tracklet array indices.
%      algorithm       - A string choosing which algorithm to use.
%          Possibilities are: 'MDS-MAP', 'SDP', 'LM-nllsq'
%
%  Outputs:
%      estimated_camers - An array of camera structs as estimated by the
%          algorithm. This is the "end product." Same format as returned
%          by generate_fake_groundtruth_cameras.
%      camera_relation_votes_and_centers - The camera relation votes
%          (theta, r, theta) and the centers chosen.
%          These are returned for plotting / data analysis purposes.
%          Structure format:
%              .votes         - A num_cameras x num_cameras cell array,
%                               each cell is a list of rows in format
%                               (theta_i, r, theta_j). These are all the
%                               votes from all the correspondences.
%              .centers.theta - A num_cameras x num_cameras array of
%                               angles, element i,j is the theta_i value of
%                               (theta_i, r, theta_j).
%              .centers.r     - A num_cameras x num_cameras array.
%                               Element i,j is the r value of
%                               (theta_i, r, theta_j). Note, symmetric.
%

num_cameras = correspondences.num_cameras;
tracklets = correspondences.tracklets_cam_coords;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tracklet Pre-Processing - Offset tracklets by camera observation
% centroids.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Loop through each tracklet, get all tracklet x,y points, organized
%by camera
xy_values = cell(num_cameras,1);
for i=1:length(tracklets)
    t = tracklets{i};
    cam = t.cam_num;
    xy_values{cam} = [xy_values{cam}; t.path(:,1:2)];
end
%compute centroid for each camera
for i=1:num_cameras
    xy = xy_values{i};
    centroids(i,1) = mean(xy(:,1));
    centroids(i,2) = mean(xy(:,2));
    %centroids(i,:) = [0 0]; %Hack to turn off centroids
end

% %Display all xy and the centroid
% for i=1:1
%     %subplot(1,2,i);
%     hold on;
%     xy = xy_values{i};
%     plot(xy(:,1),xy(:,2),'.');
%     plot(centroids(i,1),centroids(i,2),'rx');
% end

% Adjust all tracklets to remove the offset of the centroid (i.e.
% center the data at the orign)
for i=1:length(tracklets)
    t = tracklets{i};
    cam_num = t.cam_num;
    t.path(:,1) = t.path(:,1) - centroids(cam_num,1);
    t.path(:,2) = t.path(:,2) - centroids(cam_num,2);
    centered_tracklets{i} = t;
end
clear t;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tracklet Processing - the actual algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This will store lists of camera relation votes for every pairing
%of cameras. Self-pairings should be empty. Array should be filled 
%upper triangular - i.e. always pair the cameras lowest index to the
% higher index, i.e. c2->c5, NOT c5->c2.
all_centered_camera_relation_votes = cell(num_cameras, num_cameras);

% Loop through all correspondences (pairings of tracklets).
for p=1:length(correspondences.tracklet_pairings)
    pair_indices = correspondences.tracklet_pairings(p,:);
        
    %We want the lower number camera first. Swap pair if need be.
    [~,i] = sort([centered_tracklets{pair_indices(1)}.cam_num, ...
                  centered_tracklets{pair_indices(2)}.cam_num],'ascend');
    pair_indices = pair_indices(i);

    %Do this pair now - if from different cameras:
    c1 = centered_tracklets{pair_indices(1)}.cam_num;
    c2 = centered_tracklets{pair_indices(2)}.cam_num;
    if c1 ~= c2
        %Feed them in to get the best camera relation from cam 1 to 2
        % Important: the first arg is tracklet from first cam
        [theta1,r,theta2] = calculate_camera_relation( ...
                  centered_tracklets{pair_indices(1)},...
                  centered_tracklets{pair_indices(2)});
        %Only keep valid relations - if it's not valid its NaN
        if ~isnan(r)
            all_centered_camera_relation_votes{c1,c2} = ...
                [all_centered_camera_relation_votes{c1,c2}; theta1 r theta2];
        end
    end
end    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate cluster centers for all camera pairs (except self-pairs)
% - store in a matrix, like a graph.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
estimates_r     = inf*ones(num_cameras);
estimates_theta = inf*ones(num_cameras);
pairwise_centered_camera_distance_estimates = zeros(0,3);
pairwise_centered_camera_angle_estimates = zeros(0,3);
for i=1:num_cameras
    estimates_r(i,i) = 0;
    estimates_theta(i,i) = 0; %meaningless?
    for j=(i+1):num_cameras
        e = estimate_parameters_3(all_centered_camera_relation_votes{i,j},200,1);
        %Only create an estimate if we found a good vote
        if ~isempty(e)
            estimates_theta(i,j) = e(1);
            estimates_r    (i,j) = e(2);
            estimates_r    (j,i) = e(2);
            estimates_theta(j,i) = e(3);
            pairwise_centered_camera_distance_estimates = ...
                          [pairwise_centered_camera_distance_estimates; ...
                           i, j, estimates_r(i,j)];
            pairwise_centered_camera_angle_estimates = ...
                          [pairwise_centered_camera_angle_estimates; ...
                           i, j, e(1)];
            pairwise_centered_camera_angle_estimates = ...
                          [pairwise_centered_camera_angle_estimates; ...
                           j, i, e(3)];
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Position and Angle Solving
% - Note that position methods below are optimal up to a reflection,
%   so calculate both, then pick the one that gives the best angles.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%
% Calculate Best Positions
%

switch algorithm
    case 'MDS-MAP'
        estimated_locations_unreflected = cam_pos_solver_MDS_MAP(...
                      num_cameras, pairwise_centered_camera_distance_estimates);
    case 'SDP'
        estimated_locations_unreflected ...
            = cam_pos_solver_SDP1_alternating_anchors( ...
                      num_cameras, pairwise_centered_camera_distance_estimates);
    case 'LM-nllsq'
        estimated_locations_unreflected = cam_pos_solver_LM_nllsq( ...
                      num_cameras, pairwise_centered_camera_distance_estimates);
    otherwise
        error(['Unknown algorithm!: ' algorithm]);
end

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
                                      pairwise_centered_camera_angle_estimates );
estimated_angles_reflected = cam_angle_from_pos_solver( ...
                                      num_cameras, ...
                                      estimated_locations_reflected, ...
                                      pairwise_centered_camera_angle_estimates );

%
%Compute the angle cost function for both cases
%
cost_unreflected = calculate_camera_angles_cost(...
                                   pairwise_centered_camera_angle_estimates, ...
                                   estimated_angles_unreflected, ...
                                   estimated_locations_unreflected);
cost_reflected = calculate_camera_angles_cost(...
                                   pairwise_centered_camera_angle_estimates, ...
                                   estimated_angles_reflected, ...
                                   estimated_locations_reflected);

%
% Keep the results with the best (lowest) cost function.
%
if cost_reflected < cost_unreflected
    %fprintf('Keeping reflected\n');
    estimated_locations = estimated_locations_reflected;
    estimated_angles = estimated_angles_reflected;
else
    %fprintf('Keeping unreflected\n');
    estimated_locations = estimated_locations_unreflected;
    estimated_angles = estimated_angles_unreflected;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create results structure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%TODO DELETE THIS??? This poly is here just to be nice.
fov_poly.x = [0 20 20 0];
fov_poly.y = [0 -6  6 0];

%Make an array of estimated cameras, making sure to undo the centering
% step where we un-offset the centroid
for c=1:num_cameras
    cam.fov_poly_rel = fov_poly;
    a = estimated_angles(c);
    rotated_centroid = [cos(a) -sin(a); sin(a) cos(a)] * centroids(c,:)';
    cam.calib.x = estimated_locations(c,1) - rotated_centroid(1);
    cam.calib.y = estimated_locations(c,2) - rotated_centroid(2);
    cam.calib.theta = estimated_angles(c);
    estimated_cameras(c) = camera_put_in_world(cam);
end

%Camera relation votes and centers
camera_relation_votes_and_centers.votes = all_centered_camera_relation_votes;
camera_relation_votes_and_centers.centers.theta = estimates_theta;
camera_relation_votes_and_centers.centers.r     = estimates_r;

end