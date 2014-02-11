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
[ tracklets, tracks_with_tracklets ] = generate_tracklets( cameras, 100 );

%Plot them
for i=1:length(tracks_with_tracklets)
    track_with_tracklets = tracks_with_tracklets{i};
    track = track_with_tracklets.track;
    %Plot track
    plot(track(:,1),track(:,2),'-g');
    %%%plot(track(:,1),track(:,2),'.b');
    %Plot starting point
    plot(track(1,1),track(1,2),'ob');
    
    %Plot tracklets
    for j=1:length(tracklets)
        tracklet = tracklets{j};
        %Plot tracklet
        plot(tracklet.path(:,1),tracklet.path(:,2),'-r');
        %Plot tracklet starting point
        plot(tracklet.path(1,1),tracklet.path(1,2),'om');
    end
end

