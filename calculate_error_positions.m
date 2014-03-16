function [ sse ] = calculate_error_positions( ground_truth, estimated_cameras )
%calculate_error_positions Calculate error of the positions, by first
%finding the best fit translation+rotation using the procrustes transform,
%and then calculating the sum squared error.
% Input:
%  ground_truth is a structure with a field .cameras that contains the
%       actual cameras.
%  estimated_cameras - An array of the estimated cameras (structs).
% Output:
%  sse - the sum of squared error


% We want to line up the estimated locations as well as we can with the
% actual locations, so apply a euclidian transform using the procrustes
% transform

%come up with camera coodinates

groundtruth_locations = -ones(length(ground_truth.cameras),2);
for c=1:length(ground_truth.cameras)
    groundtruth_locations(c,1) = ground_truth.cameras(c).calib.x;
    groundtruth_locations(c,2) = ground_truth.cameras(c).calib.y;
end


estimated_locations = -ones(length(estimated_cameras),2);
for c=1:length(ground_truth.cameras)
    estimated_locations(c,1) = estimated_cameras(c).calib.x;
    estimated_locations(c,2) = estimated_cameras(c).calib.y;
end

%Use the procrustes transform to get the best alignment without scaling
% or reflection.
[proc_error, aligned_estimated_locations, proc_transform] ...
            = procrustes(groundtruth_locations,estimated_locations, ...
                         'Scaling',false,'Reflection',false);

%Calculate the sum-of-squares error of each coordinate.
% - it ends up being the sum of squares of every matrix element,
%   i.e. the frobenius norm squared
sse = norm(groundtruth_locations - aligned_estimated_locations, 'fro')^2;
 
end