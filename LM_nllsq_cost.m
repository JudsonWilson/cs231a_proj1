function [ residuals ] = LM_nllsq_cost( positions_stacked )
%LM_NLLSQ_COST Cost function unsed by LMFnlsq solver in the function
%  cam_pos_solver_LM_nllsq.
%
%This cost function calculates a cost for each camera distance that
%we are using as an optimization criteria. If we have a distance contraint
%r_ij between cameras i and j located at x_i and x_j, the cost residual
%for this distance is:
%   abs(r_ij norm(x_i - x_j))
%
%Input is the (x,y) positions of each camera, stacked into a tall vector,
%for compatibility with the LMnlsq solver.
%
coords = reshape(positions_stacked, 2,length(positions_stacked)/2);

%This global is needed to get the distance estimates into this function,
%I don't think LMnlsq supports passing through parameters.
global glob_camera_distance_estimates;

c = glob_camera_distance_estimates;
num_constraints = size(glob_camera_distance_estimates,1);

residuals = -ones(num_constraints,1);

% One residual per constraint, equal to the linear error of the
% positions with respect to each constraint. Note that the LM non-linear
% least squares solver will square these residuals and add them.
for i=1:num_constraints
    residuals(i) = c(i,3) - norm( coords(:,c(i,1)) - coords(:,c(i,2)));
end

%Debug print: this should match the error
%norm(residuals)^2

end

