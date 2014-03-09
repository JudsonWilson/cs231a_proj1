function [ cost ] = calculate_camera_angles_cost( ...
                                   pairwise_camera_angle_estimates, ...
                                   estimated_camera_angles, ...
                                   estimated_camera_locations)
%CALCULATE_CAMERA_ANGLES_COST Calculates the total camera cost for
%the estimated angles. I.e, very simplified:
%   sum_ij( (theta_pairwise_ij - estimated_theta_i)^2 )
%Note we may not have a constraint for all camera pairs, only the ones
%in the pairwise_camera_angle_estimates array are included.
%
% Inputs:
%   - pairwise_camera_angle_estimates - Rows of 
%             [cam_num_1, cam_num_2, angle], one row per estimate.
%             Each estimate adds a cost term!
%   - estimated_camera_angles - A simple list of angles, one per camera.
%   - estimated_camera_locations - Rows of x,y locations, the first row
%             must correspond to camera 1, etc.
% Outputs:
%   - cost - The calculated cost.
%

num_cameras = length(estimated_camera_angles);

%To convert from pairwise camera angles to absolute angle, we add the
%angle of each pairwise chord to this relative angle. We call these
%chords the "structure" angle.

%Calculate all angles of all the chords between camera pairs.
% The "structure" is the wireframe connecting each pair of points.
structure_angles = zeros(num_cameras);
for i=1:num_cameras
    for j=1:num_cameras
        if i==j; continue; end;
        delta = estimated_camera_locations(j,:)...
              - estimated_camera_locations(i,:);
        structure_angles(i,j) = atan2(delta(2),delta(1));
    end
end

%For each camera, take all the pairwise chord camera angles (from the
%camera_relation estimates), add the corresponding structure angles
%to make all of these angles relative to the x-axis in world coords,
%then add to the cost term.
cost = 0;
for c1=1:num_cameras
    %Loop through all the camera_angle_estimates and find ones for this
    %camera. Append these votes to the votes array
    for i=1:length(pairwise_camera_angle_estimates)
       if pairwise_camera_angle_estimates(i,1) == c1
           c2 = pairwise_camera_angle_estimates(i,2);
           absolute_pairwise_angle ...
                          = pairwise_camera_angle_estimates(i,3) ...
                          + structure_angles(c1,c2);
           
           angle_difference = absolute_pairwise_angle ...
                              - estimated_camera_angles(c1);
                          
            if isnan(angle_difference)
                c1=c1 %breakpoint here
            end
           
           %Make the angle difference be on the range [-pi, pi]
           while angle_difference > pi
               angle_difference = angle_difference - 2*pi;
           end
           while angle_difference < -pi
               angle_difference = angle_difference + 2*pi;
           end
           
           cost = cost + (angle_difference)^2;
       end
    end
end

%Done calculating cost

end

