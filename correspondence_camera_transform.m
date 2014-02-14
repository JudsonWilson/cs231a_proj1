function [x,y,theta] = correspondence_camera_transform(tracklet1, tracklet2)

%
%Filter and get info about both tracklets
%

%For now we don't care wether this is the beginning or the end of
%the tracklet because they are both the same.
[ x1,y1,theta1,s1, t1, ~, ~, ~, ~, ~ ] = filtered_tracklet_params( tracklet1 );
[ x2,y2,theta2,s2, t2, ~, ~, ~, ~, ~ ] = filtered_tracklet_params( tracklet2 );

% 1 - Filter out tracklets with only 1 point, since we cannot get an
% angle from these. We could get a distance, but that isn't helpful
% in the current method
if isnan(s1) || isnan(s2)
    x=NaN; y=NaN; theta=NaN;
    return
end
%elseif isnan(s1)
%    s_avg = s2;
%elseif isnan(s2)
%    s_avg = s1;
%else
    s_avg = mean([s1,s2]);
%end

% 2 - trace line in camera 1 (either forward or backward in time) from 
% start/end of tracklet to the point where the end/start of the tracklet
% in camera 2 should be
t_elapsed = t2 - t1;

x2_target = x1 + t_elapsed*s_avg*cos(theta1);
y2_target = y1 + t_elapsed*s_avg*sin(theta1);

% 3 - figure out the rotation that puts the directions in the correct
% direction.
delta_theta = theta1 - theta2;
delta_theta = mod(delta_theta + pi, 2 * pi) - pi;

% 4 - Apply the rotation to the points in camera 2, such that the line is now
% parallel but the points are now in a new location
X2_temp = [cos(delta_theta) -sin(delta_theta); ...
                      sin(delta_theta)  cos(delta_theta)] * [x2; y2];

% 5 - figure out the translation that puts rotated point 2 at the target
delta_x2 = x2_target - X2_temp(1);
delta_y2 = y2_target - X2_temp(2);

%Output - apply this to camera 2 to put it in the right place/angle.
x = delta_x2;
y = delta_y2;
theta = delta_theta;
end
