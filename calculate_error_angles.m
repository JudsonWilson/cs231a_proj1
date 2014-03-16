function [ sse ] = calculate_error_angles( ground_truth, estimated_cameras)
%CALCULATE_ERROR_ANGLES Calculates the total sum of squared cost for the
% camera angles, independent of camera relative positioning. The underlying
% idea is that we are calculating the sum of squared differences between
% the camera angles in the ground truth and the estimate, in the optimal
% rigid rotation of the estimated cameras (because our estimate is known up
% to a rotation).
%
% We approximate the optimal rotation by the the mean difference, using our
% special mean_angle formulation (remember, angle is periodic, so finding
% the mean is tricky).


angles_groundtruth = [];
for i=1:length(ground_truth.cameras)
    angles_groundtruth = [angles_groundtruth; ...
                          ground_truth.cameras(i).calib.theta];
end

angles_estimated = [];
for i=1:length(estimated_cameras)
    angles_estimated   = [angles_estimated; ...
                          estimated_cameras(i).calib.theta];
end

delta_angles = angles_groundtruth - angles_estimated;
delta_angles = mod(delta_angles, 2*pi); %easier to debug

%Find the optimal solid rotation to rotate all the estimated cameras
%as close to the ground truth as possible
optimal_rotation = mean_angle(delta_angles);

%Rotate the estated cameras
angles_estimated_optimal = angles_estimated + optimal_rotation;

%Find the individual error terms
delta_angles_optimal = angles_groundtruth - angles_estimated_optimal;

%Shift everyting to the period centered at 0 for the lowest square error
delta_angles_optimal = mod(delta_angles_optimal, 2*pi);
delta_angles_optimal(delta_angles_optimal > pi) ...
    = delta_angles_optimal(delta_angles_optimal > pi) - 2*pi;


%return the sum of squares
sse = delta_angles_optimal(:)' * delta_angles_optimal(:);

end
