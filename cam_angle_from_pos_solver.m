function [ estimated_angles ] = cam_angle_from_pos_solver( num_cameras,...
                                       estimated_locations,...
                                       pairwise_camera_angle_estimates )
%CAM_ANGLE_FROM_POS_SOLVER Solves camera angles from estimated positions
% and angle estimates. Used for "position, then angles" solution
% approaches.
%
% Inputs:
%   num_cameras - Number of cameras.
%   estimated_locations - n x 2 list of camera location estimates. These
%       positions will be used in conjunction with pairwise angles to
%       come up with the estimate.
%   pairwise_camera_angle_estimates - Pairwise angle estimates, in the
%       format rows of [cam_num_a, cam_num_b, angle]. This is the angle of
%       camera_a from the chord drawn from camera_a to camera_b.
%
% Ouptuts:
%   estimated_angles - Angles of each camera in world coordinates, as a
%       simple 1D array, or list.

%Calculate all angles of all the chords between camera pairs.
% The "structure" is the wireframe connecting each pair of points.
structure_angles = zeros(num_cameras);
for i=1:num_cameras
    for j=1:num_cameras
        if i==j; continue; end;
        delta = estimated_locations(j,:) - estimated_locations(i,:);
        structure_angles(i,j) = atan2(delta(2),delta(1));
    end
end

%For each camera, take all the votes for camera angle (from the
%camera_relation estimates), add the corresponding structure angles
%to make all of these angles relative to the x-axis in world coords,
%then average them.
estimated_angles = NaN*ones(num_cameras,1);
for c1=1:num_cameras
    votes = [];
    %Loop through all the camera_angle_estimates and find ones for this
    %camera. Append these votes to the votes array
    for i=1:length(pairwise_camera_angle_estimates)
       if pairwise_camera_angle_estimates(i,1) == c1
           c2 = pairwise_camera_angle_estimates(i,2);
           votes(end+1) = pairwise_camera_angle_estimates(i,3) ...
                          + structure_angles(c1,c2);
       end
    end
    %Take the mean angle (using the version of the mean for angles, which
    % are periodic, so require special treatment).
    estimated_angles(c1) = mean_angle(votes);
end

%Done calculating estimated_angles.

end

