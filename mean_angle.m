function [ mean_angle ] = mean_angle( angles )
%MEAN_ANGLE Given an array of angles in radians, computes a mean.
% Using method from Mean of circular quantities. Idea is to convert angles
% to points on a unit circle, find centroid of points, find the angle of
% that point.

angles = angles(:);

mean_angle = atan2( mean(sin(angles)), mean(cos(angles)));




end

