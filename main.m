clear; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Option settings
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Set this to 1 to see what happens with one set of correspondences. Set
% to 100-1000 to start seeing statistically meaningful results.
number_of_correspondences = 100;
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


%General camera field of view polygon.
% Pointing in the x-axis direction, for an (x,y,theta)=(0,0,0)
fov_poly.x = [0 20 20 0];
fov_poly.y = [0 -6  6 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Make 2 cameras
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Camera 1
camera.fov_poly_rel = fov_poly;
camera.calib.x = 15;
camera.calib.y = -5;
camera.calib.theta = pi/4; %45 degrees
camera = camera_put_in_world(camera);
cameras(1) = camera;
%Camera 2
camera.calib.x = 50;
camera.calib.y = 20;
camera.calib.theta = pi*140/100; 
camera = camera_put_in_world(camera);
cameras(2) = camera;

% Show the two cameras

figure(1);
clf;
plot_poly(cameras(1).gen.fov_poly_world);
hold on;
plot_poly(cameras(2).gen.fov_poly_world);
axis equal;

%Plot a bound-box
bound_box = cameras_bound_box(cameras);
plot( [bound_box.x(1); bound_box.x(2); bound_box.x(2); bound_box.x(1); bound_box.x(1)], ...
      [bound_box.y(1); bound_box.y(1); bound_box.y(2); bound_box.y(2); bound_box.y(1)], ...
      'r');


%Get tracklets and tracks
[ tracklets, correspondences_lists, tracks_with_tracklets ] ...
    = generate_tracklets_advanced( cameras, number_of_correspondences, ...
       track_velocity_factor, track_non_constant_factor, ...
       track_observation_variance_scale, ...
       multiple_objects_collision_percentiles, ...
       multiple_correspondence_percentage_single_tracklets );

%Plot tracks
for i=1:length(tracks_with_tracklets)
    track_with_tracklets = tracks_with_tracklets{i};
    track = track_with_tracklets.track;
    %Plot track
    plot(track(:,1),track(:,2),'-g');
    plot(track(:,1),track(:,2),'.b');
    %Plot starting point
    plot(track(1,1),track(1,2),'ob');
end
%Plot tracklets
for j=1:length(tracklets)
    tracklet = tracklets{j};
    %Plot tracklet
    plot(tracklet.path(:,1),tracklet.path(:,2),'-r');
    plot(tracklet.path(:,1),tracklet.path(:,2),'.m');
    %Plot tracklet starting point
    plot(tracklet.path(1,1),tracklet.path(1,2),'om');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tracklet Processing - the actual algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This will hold the votes for the camera positions, as column lists
% as (x,y,theta)
relative_camera_position_votes = zeros(0,3);

%At the time of writing this comment, the data is a huge list of tracklets,
%and a cell array containing lists of tracklet correspondences. We will
%process all combinations of pairs in these lists.

%In the future, the algorithm will probabily not have these lists, but
% will need to derive them on the fly while reading in tracklets one
% timestep at a time.


% Loop through all correspondences. Each iteration of this loop processes
% a list of tracklet correspondences that all happened in a short time
% window.
for i=1:length(correspondences_lists)
    correspondences_list_now = correspondences_lists{i};
    
    %We will do every pair of tracklets in this correspondence list
    correspondence_pairs = nchoosek(correspondences_list_now, 2);
    for p=1:size(correspondence_pairs,1)
        pair_indices = correspondence_pairs(p,:);
        
        %We want the lower number camera first. Swap pair if need be.
        [~,i] = sort([tracklets{pair_indices(1)}.cam_num, tracklets{pair_indices(2)}.cam_num], 'ascend')
        pair_indices = pair_indices(i);
        
        %Do this pair now - if from different cameras:
        if tracklets{pair_indices(1)}.cam_num ~= tracklets{pair_indices(2)}.cam_num
            %Feed them in to get the implied camera transform from cam 1 to cam 2
            % Important: the first arg is tracklet from cam 1, second from cam 2
            transformed_tracklets_1 = transform_tracklets_world_to_camera(cameras(1), tracklets{pair_indices(1)});
            transformed_tracklets_2 = transform_tracklets_world_to_camera(cameras(2), tracklets{pair_indices(2)});
            [x,y,theta2] = correspondence_camera_transform(transformed_tracklets_1, ...
                                                          transformed_tracklets_2);
            r = norm([x y]);
            theta1 = -atan2(y, x);
            
            %[x,y,theta] = correspondence_camera_transform(tracklets{pair_indices(1)}, ...
            %                                              tracklets{pair_indices(2)});
            if ~isnan(x)
                relative_camera_position_votes = [relative_camera_position_votes; theta1 r theta2];
            end
        end
    end
end    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the relationships
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(2);
clf;
subplot(2,2,1)
plot(relative_camera_position_votes(:,1), relative_camera_position_votes(:,2),'o');
hold on;
plot(mean(relative_camera_position_votes(:,1)), mean(relative_camera_position_votes(:,2)),'xr');
xlabel('\theta_1'); ylabel('r');

subplot(2,2,2)
plot(relative_camera_position_votes(:,2), relative_camera_position_votes(:,3),'o');
hold on;
plot(mean(relative_camera_position_votes(:,2)), mean_angle(relative_camera_position_votes(:,3)),'xr');
xlabel('r'); ylabel('\theta_2');

subplot(2,2,3)
plot(relative_camera_position_votes(:,3), relative_camera_position_votes(:,1),'o');
hold on;
plot(mean_angle(relative_camera_position_votes(:,3)), mean(relative_camera_position_votes(:,1)),'xr');
xlabel('\theta_2'); ylabel('\theta_1');

subplot(2,2,4)
plot3(relative_camera_position_votes(:,1), ...
      relative_camera_position_votes(:,2), ...
      relative_camera_position_votes(:,3),'o');
hold on;
plot3(mean      (relative_camera_position_votes(:,1)), ...
      mean      (relative_camera_position_votes(:,2)), ...
      mean_angle(relative_camera_position_votes(:,3)),'xr');
xlabel('\theta_1'); ylabel('r'); zlabel('\theta_2');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% print some basic statistics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Mean - this is probably where we would place the camera (0,0,0 is
% correct)
mean_vote = [      mean(relative_camera_position_votes(:,1)), ...
                   mean(relative_camera_position_votes(:,2)), ...
             mean_angle(relative_camera_position_votes(:,3))]
%RMS Error Values (note that x,y,theta results should equal zero, because
% camera is already in the correct place, and points are all in same 3d
% coordinate space).
rms_x     = rms(relative_camera_position_votes(:,1))
rms_y     = rms(relative_camera_position_votes(:,2))
rms_theta = rms(relative_camera_position_votes(:,3))
