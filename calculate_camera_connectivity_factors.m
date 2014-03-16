function [ connections, potential_connections ] ...
     = calculate_camera_connectivity_factors( camera_relation_votes_and_centers)
%CALCULATE_CAMERA_CONNECTIVITY_FACTORS Calculate the number of camera
%pairings for which we estimated a relation between, and the number we
%potentially could have. 

%the r field is basically a distance matrix, where each row/col represents
% a camera, and the value is Inf if no relationship was found.
r = camera_relation_votes_and_centers.centers.r;

num_missing = sum(sum(isinf(r)))/2;

num_cameras = size(r,1);
potential_connections = (num_cameras + 1) * num_cameras / 2 - num_cameras;

connections = potential_connections - num_missing;

end
