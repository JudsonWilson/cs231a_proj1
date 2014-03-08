function [ cost ] = calculate_camera_positions_cost( camera_constraints, estimated_camera_locations )
%CALCULATE_CAMERA_POSITIONS_COST Calculates the total distance error for
%the estimted locations. I.e:
%   sum_ij( (||x_i - x_j|| - r_ij)^2 )
%Note we may not have a constraint for all camera pairs, only the ones
%in the camera_constraints array are included.
%
% Inputs:
%   - camera_constraints - Rows of [cam_num_1, cam_num_2, distance]
%             one row per constraint. Each constraint adds a cost term!
%
%   - estimated_camera_locations - Rows of x,y locations, the first row
%             must correspond to camera 1, etc.
% Outputs:
%   - cost - The calculated cost.
%

c = camera_constraints;
X = estimated_camera_locations';

if size(X,1) ~= 2
    error('estimated_camera_locations should be n-by-2');
end
if size(c,2) ~= 3
    error('camera_constraints should be n-by-3');
end

cost = 0;
for i = 1:size(camera_constraints,1)
    cost = cost + (norm(X(:,c(i,1)) - X(:,c(i,2)))  -  c(i,3))^2;
end

end

