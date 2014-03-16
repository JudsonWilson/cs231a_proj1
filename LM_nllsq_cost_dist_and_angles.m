function [ residuals ] = LM_nllsq_cost_dist_and_angles( positions_angles_stacked )
%LM_NLLSQ_COST_DIST_AND_ANGLES Cost function unsed by LMFnlsq solver in the
%  function cam_extcal_solver_LM_nllsq_dist_and_angles.
%
%This cost function calculates a cost for each camera distance that
%we are using as an optimization criteria, as well as each angle.
%
%If we have a distance contraint
%r_ij between cameras i and j located at x_i and x_j, the cost residual
%for this distance is:
%   abs(r_ij norm(x_i - x_j))
%
%If we have an angle contraint alpha_ij on camera_i from the chord between
%between cameras i and j located at (x_i,y_i) and (x_j,y_j), the cost
%residual for this distance is:
%   abs(alpha_ij - atan2(y_j-y_i, x_j-x_i))
%
%Input is the (x,y) positions of each camera, stacked into a tall vector,
%followed by the angles, for compatibility with the LMnlsq solver.
%

%Top 2/3 of the stacked input vector is positions (because we have x,y
%coords, and all that is left is angles, so 2/3).
positions_stacked = positions_angles_stacked(...
                      1:(size(positions_angles_stacked,1)*2/3));
%Bottom 1/3 of the stacked input vector is positions (because we have x,y
%coords, and all that is left is angles, so 2/3).
angles = positions_angles_stacked(...
                      (size(positions_angles_stacked,1)*2/3+1):end);

coords = reshape(positions_stacked, 2, length(positions_stacked)/2);

%This global is needed to get the distance estimates into this function,
%I don't think LMnlsq supports passing through parameters.
global glob_camera_distance_estimates;
global glob_camera_angle_estimates;

d = glob_camera_distance_estimates;
num_d_constraints = size(glob_camera_distance_estimates,1);

a = glob_camera_angle_estimates;
num_a_constraints = size(glob_camera_angle_estimates,1);

residuals = -ones(num_d_constraints+num_a_constraints,1);

% One residual per distance constraint, equal to the linear error of the
% positions with respect to each constraint. Note that the LM non-linear
% least squares solver will square these residuals and add them.
for i=1:num_d_constraints
    residuals(i) = d(i,3) - norm( coords(:,d(i,1)) - coords(:,d(i,2)));
end

% One residual per angle constraint, equal to the linear error of the
% angles with respect to each constraint. Note that the LM non-linear
% least squares solver will square these residuals and add them.
for i=1:num_a_constraints
    r = coords(:,a(i,2)) - coords(:,a(i,1));
    error = (angles(a(i,1)) - atan2(r(2),r(1))) - a(i,3);
    %Make it so error near 2*pi is actually error near 0 (since it is!)
    %This should lead to better solver descent.
    error = mod(error,2*pi);
    if error > pi
        error = error - 2*pi;
    end
    residuals(i+num_d_constraints) = error;
end


%Debug print: this should match the error
%norm(residuals)^2

end

