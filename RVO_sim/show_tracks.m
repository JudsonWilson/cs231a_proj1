%% Plot Camera Boxes
close all
clear
clc

% Camera Locations
x0 = [17,27,27];
y0 = [17,33,17];
theta = [45,225,90].*pi/180;
rot = @(theta) [cos(theta),-sin(theta);sin(theta),cos(theta)];

% Camera Polygon
v0 = [9;1];
v1 = [17.5;5.5];
v2 = [17.5;-5.5];
v3 = [9;-1];
poly = [v0,v1,v2,v3,v0];
% Camera Triangle
v0_t = [0;0];
v1_t = [17.5;5.5];
v2_t = [17.5;-5.5];
tri = [v0_t,v1_t,v2_t,v0_t];

% Plot
figure(1)
axis([0 50 16 36])
hold on
for i = 1:length(x0)
    % Rotate Poly by Theta
%     tri_i = rot(theta(i))*tri + repmat([x0(i);y0(i)],1,size(tri,2));
%     plot(tri_i(1,:),tri_i(2,:))
    poly_i = rot(theta(i))*poly + repmat([x0(i);y0(i)],1,size(poly,2));
    plot(poly_i(1,:),poly_i(2,:),'-r')
%     waitforbuttonpress
    
end

%% Load Tracks
filenames = {'out.csv','out_cam0.csv','out_cam1.csv','out_cam2.csv'};

% Load Ground Truth
tracks = dlmread(filenames{1},' ');
% Skip Last Space
tracks = tracks(:,1:end-1);

% Load Cam Data
cam0_tracks = dlmread(filenames{2},' ');
cam1_tracks = dlmread(filenames{3},' ');
cam2_tracks = dlmread(filenames{4},' ');

%% Plot Tracks
colors = repmat('rgbcymk',1,20);

[num_tracks,all_data_len] = size(tracks(:,4:end));
for i = 1:num_tracks
    disp(sprintf('Track: %d',i))
    % Get ground truth track
    len_data_valid = tracks(i,3);
    track = tracks(i,4:end);
    % Reshape
    track_line = reshape(track,3,all_data_len/3);
    % Take only the valid data
    track_line_valid = track_line(:,1:len_data_valid);

    % Plot ground truth
    hg = plot(track_line_valid(1,:),track_line_valid(2,:),'-b');%sprintf('-%c',colors(i)));
    waitforbuttonpress    
    
    % Plot Cam 0 overlap
    cam0_inds = find(cam0_tracks(:,4) == i-1);
    if (~isempty(cam0_inds))
        cam0_pts = cam0_tracks(cam0_inds,:)';
        cam0_pts_glob = rot(theta(1))*cam0_pts(1:2,:) + repmat([x0(1);y0(1)],1,size(cam0_pts,2));
        h0 = plot(cam0_pts_glob(1,:),cam0_pts_glob(2,:),'-g');%sprintf('-%c',colors(i+1)));
    else
        disp('No Overlap for Cam 0');
    end
    waitforbuttonpress
    
    % Plot Cam 1 overlap
    cam1_inds = find(cam1_tracks(:,4) == i-1);
    if (~isempty(cam1_inds))
        cam1_pts = cam1_tracks(cam1_inds,:)';
        cam1_pts_glob = rot(theta(2))*cam1_pts(1:2,:) + repmat([x0(2);y0(2)],1,size(cam1_pts,2));
        h1 = plot(cam1_pts_glob(1,:),cam1_pts_glob(2,:),'-y');%sprintf('-%c',colors(i+2)));
    else
        disp('No Overlap for Cam 1');
    end
    waitforbuttonpress
    
    % Plot Cam 2 overlap
    cam2_inds = find(cam2_tracks(:,4) == i-1);
    if (~isempty(cam2_inds))
        cam2_pts = cam2_tracks(cam2_inds,:)';
        cam2_pts_glob = rot(theta(3))*cam2_pts(1:2,:) + repmat([x0(3);y0(3)],1,size(cam2_pts,2));
        h2 = plot(cam2_pts_glob(1,:),cam2_pts_glob(2,:),'-m');%sprintf('-%c',colors(i+3)));
    else
        disp('No Overlap for Cam 2');
    end
    waitforbuttonpress
    
    set(hg,'Visible','off')
    if(~isempty(cam0_inds))
        set(h0,'Visible','off')
    end
    if(~isempty(cam1_inds))
        set(h1,'Visible','off')
    end
    if(~isempty(cam2_inds))
        set(h2,'Visible','off')
    end
end










