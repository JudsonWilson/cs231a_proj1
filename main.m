clear; clc;

%Libraries of stuff I didn't make
addpath('./extlib/');

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
camera.calib.x = 10;
camera.calib.y = 20;
camera.calib.theta = pi*140/100; 
camera = camera_put_in_world(camera);
cameras(2) = camera;

% Show the two cameras

figure(1);
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
[ tracklets, tracks_with_tracklets ] = generate_tracklets( cameras, 20 );

%Plot tracks
for i=1:length(tracks_with_tracklets)
    track_with_tracklets = tracks_with_tracklets{i};
    track = track_with_tracklets.track;
    %Plot track
    plot(track(:,1),track(:,2),'-g');
    %%%plot(track(:,1),track(:,2),'.b');
    %Plot starting point
    plot(track(1,1),track(1,2),'ob');
end
%Plot tracklets
for j=1:length(tracklets)
    tracklet = tracklets{j};
    %Plot tracklet
    plot(tracklet.path(:,1),tracklet.path(:,2),'-r');
    %Plot tracklet starting point
    plot(tracklet.path(1,1),tracklet.path(1,2),'om');
end


%Trim down to just pairs - possible correspondences
tracklets_correspondences = {};
for i=1:length(tracks_with_tracklets)
    tracklets_to_check = tracks_with_tracklets{i}.tracklets;
    %If we have at least 2 tracklets bundled, we have a correspondence.
    %Bundle them in a cell array inside the larger cell array that holds
    %all these bundles.
    if length(tracklets_to_check) > 1
        %As of right now tracklets are already sorted by camera, but that
        %may not always be the case.
        tracklets_to_check = sort_tracklets_by_camera(tracklets_to_check);
        tracklets_correspondences = [tracklets_correspondences {tracklets_to_check}];
    end
end

%Generate implied relative camera position/angle for each correspondence
for i=1:length(tracklets_correspondences)
    %Get next cell array of corresponding tracklets (a correspondence)
    correspondence_tracklet_group = tracklets_correspondences{i};
    %Feed them in to get the implied camera transform from cam 1 to cam 2
    % Important: the first arg is tracklet from cam 1, second from cam 2
    [x,y,theta] = correspondence_camera_transform(correspondence_tracklet_group{1}, ...
                                    correspondence_tracklet_group{2})
    
    % README!! README!! README!! README!! README!! README!! README!! README!!
    % README!! README!! README!! README!! README!! README!! README!! README!!
    % README!! README!! README!! README!! README!! README!! README!! README!!

    % At this point, x,y,theta should be close to zero, becaues the
    % camera is already in the correct place (and note the tracklets
    % are in world coordinates, not relative coordinates!)

    % README!! README!! README!! README!! README!! README!! README!! README!!
    % README!! README!! README!! README!! README!! README!! README!! README!!
    % README!! README!! README!! README!! README!! README!! README!! README!!
    % README!! README!! README!! README!! README!! README!! README!! README!!
end

%Generate implied relative camera position/angle







