clear; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Option settings
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Set this to 1 to see what happens with one set of correspondences. Set
% to 100-1000 to start seeing statistically meaningful results.
number_of_correspondences = 1000;
%number_of_correspondences = 10;

%This is sort of "starting point" for track velocity, probably
%leave it at 1 for now
track_velocity_factor = 1;

% Increasing this makes lines more curvy, and points will speed up and
% slow down. 0 = straight lines, constant velocity.
track_non_constant_factor = 1; 
% track_non_constant_factor = 0; 

% Increasing this makes observations noisier. Observations are taken
% from the curvy path and gaussian noise is added. 0 = no noise.
% 1 is my usual value.
track_observation_variance_scale = 1;
% track_observation_variance_scale = 0;

% As set, we will see correspondences of 1 path 30% of the time,
% two paths 40% of the time, and 3 paths 30% of the time. To never
% have errors, set this to 1 path only [1.01 0 0]. Errors come from
% having multiple paths that give false correspondences.
multiple_objects_collision_percentiles = [0.3 0.7 1.01];
% multiple_objects_collision_percentiles = [1.01 0 0];


%When the collision percentiles allow for multiple tracks at the same
% time, this will dictate how often in the collision that any track
% has only one tracklet.  Set this near 0 for mostly tracks that intersect
% pairs of camera views, and set it to 1 for tracks that intersect
% mostly single camera views.
multiple_correspondence_percentage_single_tracklets = 0.3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Include Paths
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('./lib/mds_map/');
addpath('./lib/LMFnlsq/');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Make Cameras
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This polygon is used for plotting the final result, NOT for anything
%related to generating/interpretting data.
esitmate_plot_fov_poly.x = [0 20 20 0];
esitmate_plot_fov_poly.y = [0 -6  6 0];

%Create an array of camera structs.
cameras = generate_fake_groundtruth_cameras(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start making tracklets / making plot of them
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1);
clf;
hold on;
[ correspondences, ground_truth ] = ...
        generate_fake_test_data(cameras,...
                                number_of_correspondences, ...
                                track_velocity_factor, ...
                                track_non_constant_factor, ...
                                track_observation_variance_scale, ...
                                multiple_objects_collision_percentiles, ...
                                multiple_correspondence_percentage_single_tracklets, ...
                                true); %with plot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tracklet Processing - the actual algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This will hold the votes for the camera positions, as column lists
% as (x,y,theta)
relative_camera_position_votes = zeros(0,3);

%This will store lists of camera relation votes for every pairing
%of cameras. Self-pairings should be empty. Array should be filled 
%upper triangular - i.e. always pair the cameras lowest index to the
% higher index, i.e. c2->c5, NOT c5->c2.
all_camera_relation_votes = cell(length(cameras), length(cameras));

% Loop through all correspondences (pairings of tracklets).
for p=1:length(correspondences.tracklet_pairings)
    pair_indices = correspondences.tracklet_pairings(p,:);
        
    %We want the lower number camera first. Swap pair if need be.
    [~,i] = sort([correspondences.tracklets_cam_coords{pair_indices(1)}.cam_num, ...
                  correspondences.tracklets_cam_coords{pair_indices(2)}.cam_num],'ascend');
    pair_indices = pair_indices(i);

    %Do this pair now - if from different cameras:
    c1 = correspondences.tracklets_cam_coords{pair_indices(1)}.cam_num;
    c2 = correspondences.tracklets_cam_coords{pair_indices(2)}.cam_num;
    if c1 ~= c2
        %Feed them in to get the best camera relation from cam 1 to 2
        % Important: the first arg is tracklet from first cam
        [theta1,r,theta2] = calculate_camera_relation( ...
                  correspondences.tracklets_cam_coords{pair_indices(1)},...
                  correspondences.tracklets_cam_coords{pair_indices(2)});
        %Only keep valid relations - if it's not valid its NaN
        if ~isnan(r)
            all_camera_relation_votes{c1,c2} = ...
                [all_camera_relation_votes{c1,c2}; theta1 r theta2];
        end
    end
end    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate cluster centers for all camera pairs (except self-pairs)
% - store in a matrix, like a graph.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
estimates_r      = inf*ones(length(cameras));
estimates_theta1 = inf*ones(length(cameras));
estimates_theta2 = inf*ones(length(cameras));
for i=1:length(cameras)
    estimates_r(i,i) = 0;
    estimates_theta1(i,i) = 0; %meaningless?
    estimates_theta2(i,i) = 0; %meaningless?
    estimates_theta2as1(i,i) = 0; %meaningless?
    for j=(i+1):length(cameras)
        e = estimate_parameters_2(all_camera_relation_votes{i,j},200,1);
        estimates_theta1(i,j) = e(1);
        estimates_r     (i,j) = e(2);
        estimates_theta2(i,j) = e(3);
    end
end
%Make the non-triangular versions of the matrices
% - r is symmetric
estimates_r              = min(cat(3,      estimates_r,      estimates_r'),[],3);
estimates_theta_combined = min(cat(3, estimates_theta1, estimates_theta2'),[],3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the Camera Relation Votes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Plot relationship votes from camera 1 to camera 2
figure(2); clf;
make_plots_camera_relation_votes    (all_camera_relation_votes{1,2});
make_plots_camera_relation_estimates(estimates_theta1(1,2), estimates_r(1,2), estimates_theta2(1,2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MDS-Map Step
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%

%Convert data to distance-constraint list format
% - rows of [ cam_num_1, cam_num_2, distance ]
% - generally speaking does not need all camera pairings
camera_distance_estimates = zeros(0,3);
for i=1:size(estimates_r,1)
    for j=i+1:size(estimates_r,2)
        camera_distance_estimates = [camera_distance_estimates; ...
                                       i, j, estimates_r(i,j)];
    end
end

%estimated_locations = cam_pos_solver_MDS_MAP(...
%                           length(cameras), camera_distance_estimates);

%estimated_locations = cam_pos_solver_SDP1_alternating_anchors( ...
%                           length(cameras), camera_distance_estimates);

estimated_locations = cam_pos_solver_LM_nllsq( ...
                           length(cameras), camera_distance_estimates);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot result
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(3); clf; hold on;

% Show the cameras
for c = 1:length(cameras)
    plot_poly(cameras(c).gen.fov_poly_world);
end
axis equal;

%Plot a bound-box
bound_box = cameras_bound_box(cameras);
plot( [bound_box.x(1); bound_box.x(2); bound_box.x(2); bound_box.x(1); bound_box.x(1)], ...
      [bound_box.y(1); bound_box.y(1); bound_box.y(2); bound_box.y(2); bound_box.y(1)], ...
      'r');

% We want to line up the estimated locations as well as we can with the
% actual locations, for display purposes - apply a euclidian transform.

%come up with camera coodinates
c_locations = -ones(length(cameras),2);
for c=1:length(cameras)
    c_locations(c,1) = cameras(c).calib.x;
    c_locations(c,2) = cameras(c).calib.y;
end

%Use the procrustes transform to get the best alignment without scaling
[~, estimated_locations] = procrustes(c_locations, estimated_locations,'scaling',false);
  
%Plot the estimated locations, rotated and shifted to align with the
%original locations, for display purposes
plot(estimated_locations(:,1), estimated_locations(:,2),'bd');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Angle Estimation Step
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Calculate all angles between lines in the MDS result structure.
structure_angles = zeros(length(cameras));
for i=1:length(cameras)
    for j=1:length(cameras)
        if i==j; continue; end;
        delta = estimated_locations(j,:) - estimated_locations(i,:);
        structure_angles(i,j) = atan2(delta(2),delta(1));
    end
end

%For each camera, find the mean angle relative to x-axis. This is the
% absolute camera angle in world coordinates.
for c1=1:length(cameras)
   votes = [];
   for c2=1:length(cameras)
       if c2==c1; continue; end;
       votes(end+1) = estimates_theta_combined(c1,c2) + structure_angles(c1,c2);
   end
   estimated_angles(c1) = mean_angle(votes);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot cameras at new locations and angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for c=1:length(cameras)
    camera.fov_poly_rel = esitmate_plot_fov_poly;
    camera.calib.x = estimated_locations(c,1);
    camera.calib.y = estimated_locations(c,2);
    camera.calib.theta = estimated_angles(c);
    estimated_cameras(c) = camera_put_in_world(camera);
end

% Show the estimated cameras
for c = 1:length(estimated_cameras)
    plot_poly(estimated_cameras(c).gen.fov_poly_world);
end


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % print some basic statistics
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% relative_camera_position_votes = implied_camera_relations{1,2};
% 
% 
% % Mean - this is probably where we would place the camera (0,0,0 is
% % correct)
% mean_vote = [      mean(relative_camera_position_votes(:,1)), ...
%                    mean(relative_camera_position_votes(:,2)), ...
%              mean_angle(relative_camera_position_votes(:,3))]
% %RMS Error Values (note that x,y,theta results should equal zero, because
% % camera is already in the correct place, and points are all in same 3d
% % coordinate space).
% rms_x     = rms(relative_camera_position_votes(:,1))
% rms_y     = rms(relative_camera_position_votes(:,2))
% rms_theta = rms(relative_camera_position_votes(:,3))
