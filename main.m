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
% Position and Angle Solving
% - Note that position methods below are optimal up to a reflection,
%   so calculate both, then pick the one that gives the best angles.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[estimated_cameras, camera_relation_votes_and_centers] ...
    = solve_cameras_extcal(correspondences, 'SDP');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the Camera Relation Votes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Plot relationship votes from camera 1 to camera 2
figure(2); clf;
make_plots_camera_relation_votes( ...
        camera_relation_votes_and_centers.votes{1,2});
make_plots_camera_relation_estimates( ...
        camera_relation_votes_and_centers.centers.theta(1,2), ...
        camera_relation_votes_and_centers.centers.r(1,2), ...
        camera_relation_votes_and_centers.centers.theta(2,1));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Results - Estimated Cameras vs Ground Truth
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
groundtruth_locations = -ones(length(cameras),2);
estimated_locations   = -ones(length(cameras),2);

for c=1:length(cameras)
    groundtruth_locations(c,1) = cameras(c).calib.x;
    groundtruth_locations(c,2) = cameras(c).calib.y;
    estimated_locations(c,1) = estimated_cameras(c).calib.x;
    estimated_locations(c,2) = estimated_cameras(c).calib.y;
end


%Use the procrustes transform to get the best alignment without scaling
% or reflection. Also do it on the reflection of the points and print
% their values. Useful to see if we chose the correct reflection or not.
[proc_error, aligned_estimated_locations, proc_transform] ...
            = procrustes(groundtruth_locations,estimated_locations, ...
                         'Scaling',false,'Reflection',false);
[proc_error_reflected] ...
            = procrustes(groundtruth_locations, [-estimated_locations(:,1), ...
                                        estimated_locations(:,2)], ...
                         'Scaling',false,'Reflection',false);
fprintf('Procustes error using chosen locations: %f,\n', proc_error);
fprintf('    and reflection of chosen locations: %f,\n', proc_error_reflected);
if proc_error <= proc_error_reflected
    fprintf('It appears we DID choose the correct reflection.\n');
else
    fprintf('It appears we did NOT choose the correct reflection.\n');
    fprintf('This is not neccessarily a bug.\n');
end    

%Convert the transform rotation matrix to an angle. Do this by rotating
%the e1 vector and finding the rotation.
display_angle = atan2(proc_transform.T(:,2)'*[1;0], ...
                      proc_transform.T(:,1)'*[1;0]);

%Plot the estimated locations, rotated and shifted to align with the
%original locations, for display purposes
plot(aligned_estimated_locations(:,1), aligned_estimated_locations(:,2),'bd');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot cameras at new locations and angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for c=1:length(cameras)
    camera.fov_poly_rel = esitmate_plot_fov_poly;
    camera.calib.x = aligned_estimated_locations(c,1);
    camera.calib.y = aligned_estimated_locations(c,2);
    camera.calib.theta = estimated_cameras(c).calib.theta + display_angle;
    aligned_cameras(c) = camera_put_in_world(camera);
end

% Show the estimated cameras
for c = 1:length(aligned_cameras)
    plot_poly(aligned_cameras(c).gen.fov_poly_world);
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
