function center = estimate_parameters_2(camera_relation_votes, num_bins, blur_factor)
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
%   - center            - 3x1 vector containing estimates for coordinate
%                         (theta1,r,theta2)
%
% num_bins = 200;
% blur_factor = 1;

% Compute Gaussian Blur (in bins)
blur = normpdf(-floor(num_bins/2):floor(num_bins/2),0,blur_factor);

angular_flag = [1,0,1]; %Says that theta1 and theta2 are angles, not linear

%Loop over 3 parameters
for i=1:3
    % Bin range
    dx = range(camera_relation_votes(:,i)/num_bins);
    % Take histogram
    [x_bins, x_bin_centers] = hist(camera_relation_votes(:,i),num_bins);
    % Blur histogram - do differently for r than for the theta1,theta2
    if angular_flag(i)
        %We want to simulate a periodic value, so make 3 copies,
        % do convolution, then grab the middle copy when done.
        x_bins_blur = conv([x_bins x_bins x_bins],blur,'same');
        x_bins_blur = x_bins_blur((num_bins+1):(2*num_bins));
    else
        %Normal convolution for non-periodic values (r)
        x_bins_blur = conv(x_bins,blur,'same');
    end
    % Take bin with highest value
    [x_bin_max, x_bin_max_ind] = max(x_bins_blur);
    % Find coordinates within most frequent bin
    x_inds = find(abs(x_bin_centers(x_bin_max_ind)-camera_relation_votes(:,i)) < dx/2);
    % Estimate parameter
    x = mean(camera_relation_votes(x_inds,i));
  
    center(i) = x;
end

end