function centroid = estimate_parameters(relative_camera_position_votes, num_bins, blur_factor)
%
% Independently estimate the relative position and angle of a pair of cameras by:
%       1) Compute 1-D Histogram
%       2) Blur histogram bins with gaussian distribution
%       3) Average parameters within maximal bin's range
%       4) Return average as parameter estimate
%
% Input(s):
%   - relative_camera_position_votes 
%                       - Input positions and angles of each tracklet
%   - num_bins          - Number of bins for histogram
%   - blur_factor       - Variance of Gaussian Blurring of bins
% Output(s)
%   - centroid          - 3x1 vector containing estimates for x,y,theta
%
% num_bins = 200;
% blur_factor = 1;

% Compute Gaussian Blur (in bins)
blur = normpdf(-floor(num_bins/2):floor(num_bins/2),0,blur_factor);

% Compute X
% Bin range
dx = range(relative_camera_position_votes(:,1)/num_bins);
% Take histogram
[x_bins, x_bin_centers] = hist(relative_camera_position_votes(:,1),num_bins);
% Blur histogram
x_bins_blur = conv(x_bins,blur,'same');
% Take bin with highest value
[x_bin_max, x_bin_max_ind] = max(x_bins_blur);
% Find x coordinates within most frequent bin
x_inds = find(abs(x_bin_centers(x_bin_max_ind)-relative_camera_position_votes(:,1)) < dx/2);
% Estimate parameter x
x = mean(relative_camera_position_votes(x_inds,1));


% Compute Y
% Bin range
dy = range(relative_camera_position_votes(:,2)/num_bins);
% Take histogram
[y_bins, y_bin_centers] = hist(relative_camera_position_votes(:,2),num_bins);
% Blur histogram
y_bins_blur = conv(y_bins,blur,'same');
% Take bin with highest value
[y_bin_max, y_bin_max_ind] = max(y_bins_blur);
% Find y coordinates within most frequent bin
y_inds = find(abs(y_bin_centers(y_bin_max_ind)-relative_camera_position_votes(:,2)) < dy/2);
% Estimate parameter y
y = mean(relative_camera_position_votes(y_inds,2));


% Compute Theta
% Bin range
dtheta = range(relative_camera_position_votes(:,3)/num_bins);
% Take histogram
[theta_bins, theta_bin_centers] = hist(relative_camera_position_votes(:,3),num_bins);
% Blur histogram
theta_bins_blur = conv(theta_bins,blur,'same');
% Take bin with highest value
[theta_bin_max, theta_bin_max_ind] = max(theta_bins_blur);
% Find theta coordinates within most frequent bin
theta_inds = find(abs(theta_bin_centers(theta_bin_max_ind)-relative_camera_position_votes(:,3)) < dtheta/2);
% Estimate parameter theta
theta = mean(relative_camera_position_votes(theta_inds,3));


% Combine into output
centroid = [x;y;theta];

end