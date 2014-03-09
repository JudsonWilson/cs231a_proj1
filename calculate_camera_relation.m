function [theta1,r,theta2] = calculate_camera_relation(tracklet1, tracklet2)
%CALCULATE_CAMERA_RELATION does what it says.
% Inputs:
%     tracklet1, tracklet2 - tracklets of an object that moved through
%          the camera that these tracklets came from. Note the tracklets
%          should be in relative coordinates.
% Outputs:
%     r - The distance between camera origins.
%     theta1, theta2 - Using a ray drawn from the origin of the respective
%     camera to the other camera, these are the angles from these rays.
%     i.e. theta1 is the angle from "the ray pointing from the first camera
%     to the second camera", to camera 1's centerline. Note these angles,
%     as defined, are the same, but swapped, if you swap cameras.

%
%Filter and get info about both tracklets
%

%For now we don't care wether this is the beginning or the end of
%the tracklet because they are both the same.
[ x1,y1,theta1r,s1, t1, ~, ~, ~, ~, ~ ] = filtered_tracklet_params( tracklet1 );
[ x2,y2,theta2r,s2, t2, ~, ~, ~, ~, ~ ] = filtered_tracklet_params( tracklet2 );

% 1 - Filter out tracklets with only 1 point, since we cannot get an
% angle from these. We could get a distance, but that isn't helpful
% in the current method
if isnan(s1) || isnan(s2)
    theta1=NaN; r=NaN; theta2=NaN;
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

x2_target = x1 + t_elapsed*s_avg*cos(theta1r);
y2_target = y1 + t_elapsed*s_avg*sin(theta1r);

% 3 - figure out the rotation that puts the directions in the correct
% direction.
delta_theta = theta1r - theta2r;

% 4 - Apply the rotation to the points in camera 2, such that the line is now
% parallel but the points are now in a new location
X2_temp = [cos(delta_theta) -sin(delta_theta); ...
                      sin(delta_theta)  cos(delta_theta)] * [x2; y2];

% 5 - figure out the translation that puts rotated point 2 at the target
delta_x2 = x2_target - X2_temp(1);
delta_y2 = y2_target - X2_temp(2);

% angle of ray 
theta_ray = atan2(delta_y2,delta_x2);

%Output - Based on a ray drawn from camera origin 1 to camera origina 2,
%  r is the distance between camera origins, theta1 is the angle of
%  camera 1 relative to the ray, and theta2 is the angle of camera2
%  relative to the opposite ray (so the result is the same if you swap
%  cameras).
theta1 = -theta_ray; %Angle from the ray
r = norm([delta_x2 delta_y2]);
%Calculate theta2 as the angle from the ray p1->p2 to the camera.
theta2 = delta_theta - theta_ray;
%Change theta2 to be the angle from the ray p2->p1 to the camera.
theta2 = -(pi - theta2);

%Force angles to the range [-pi,pi]
theta1 = mod(theta1 + pi, 2 * pi) - pi;
theta2 = mod(theta2 + pi, 2 * pi) - pi;
end
