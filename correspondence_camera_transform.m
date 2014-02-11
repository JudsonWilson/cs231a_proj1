function [x,y,theta] = correspondence_camera_transform(tracklet1, tracklet2)

%
%Filter and get info about both tracklets
%

%For now we don't care wether this is the beginning or the end of
%the tracklet because they are both the same.
[ x1,y1,theta1,s1, t1, ~, ~, ~, ~, ~ ] = filtered_tracklet_params( tracklet1 );
[ x2,y2,theta2,s2, t2, ~, ~, ~, ~, ~ ] = filtered_tracklet_params( tracklet2 );

% 1 - calculate average speed
s_avg = mean([s1,s2]);

% 2 - trace line in camera 1 (either forward or backward in time) from 
% start/end of tracklet to the point where the end/start of the tracklet
% in camera 2 should be
t_elapsed = t2 - t1;

x2_target = x1 + t_elapsed*s_avg*cos(theta1);
y2_target = y1 + t_elapsed*s_avg*sin(theta1);

% 3 - figure out the translation that puts that point 2 at the target

delta_x2 = x2_target - x2;
delta_y2 = y2_target - y2;

% 4 - figure out the rotation that puts the directions in the correct
% direction.
delta_theta = theta1 - theta2;

%Output - apply this to camera 2 to put it in the right place/angle.
x = delta_x2;
y = delta_y2;
theta = delta_theta;
end
