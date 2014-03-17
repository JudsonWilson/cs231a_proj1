function [ x1,y1,theta1,s1, t1, x2, y2, theta2, s2, t2 ] ...
    = filtered_tracklet_params( tracklet )
%FILTERED_TRACKLET_PARAMS Take a tracklet, filter it somehow, and come
% up with a start/end coordinate and time, and a velocity.

%Current method: fit a line using PCA (total least squares), assume the 
%parameters for the tracklet start/end are both the same and are the
%values at the centroid.

%Subtract the mean x,y values, chop off t
centroid = mean(tracklet.path(:,1:2),1);
centered = bsxfun(@minus,tracklet.path(:,1:2),centroid);
%Take svd
[U S V] = svd(centered);
%direction of highest singular value is the direction of the line.
direction = V(:,1);
orthog = V(:,2);

x = centroid(1);
y = centroid(2);
delta_t = tracklet.path(end,3)-tracklet.path(1,3);
t = tracklet.first_time + delta_t/2;

theta = atan2(direction(2), direction(1));

%Give the distance along the fit line from the centroid to the first point
% and last point (either positive of negative value).
d_start = dot(direction, centered(1,  :)');
d_end   = dot(direction, centered(end,:)');

%Calculate average speed on the line.
s = (d_end - d_start) / delta_t;

%Set the output values the same for the start/end of tracklet (shouldn't
% matter so long as the time stamps match the time the point was probably
% at the centroid).
x1 = x; x2 = x;
y1 = y; y2 = y;
theta1 = theta; theta2 = theta;
s1 = s; s2 = s;
t1 = t; t2 = t;


end

