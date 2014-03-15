function [center votes_in_window] = estimate_parameters_2(camera_relation_votes, num_bins, blur_factor)
%
% Jointly estimate the relative position and angles of a camera pair by:
%       1) Dividing space into subwindow bins (a 3d space).
%          - note, only cover the 75% of points closest to zero to keep
%            space from getting too large due to outliers
%       2) Putting all votes in bins.
%       3) Slide a 3x3x3 window over the subwindows so that we get
%          larger windows that overlap.
%       2) Find the peak window bin, make sure it's at contains at least a
%          certain percentage of the points (Percentage depends on the
%          number of points overall. For fewer points, we want to be
%          stricter since the statistical significance is lower, we want
%          to maintain high confidence. Also, all inputs with too few
%          points are flat rejected).
%       3) Return the median coordinates of all votes in the best window.
%
% Input(s):
%   - relative_camera_position_votes 
%                       - Input positions and angles of each tracklet
%   - num_bins          - Number of bins for histogram
%   - blur_factor       - Variance of Gaussian Blurring of bins
% Output(s)
%   - center            - 3x1 vector containing estimates for coordinate
%                         (theta1,r,theta2)
%   - votes_in_window   - a list of the camera_relation_votes indices used
%                         after voting to generate median. Mainly for
%                         plotting purposes

% The below code was found to be too ambitious, too strict.
% %Make sure there is more than 9 bins per dimension (to ensure that the
% %largest window spans no more than 30% of the voting space in that
% %dimension), less than 30 bins per dimension (seems like a good empiricle
% %amount of resolution, and won't blow up memory).
% % Within these limits, approximately pick enough bins such that there will
% % be 4 votes per bin, on average.
% %max_num_subbins = 30;
% %num_subbins_per_dim = max(9, ...
% %                      min(max_num_subbins, ...
% %                          floor((length(camera_relation_votes)/4)^(1/3))...
% %                      ) );

%Experimentally, this seems good. We want the best window to cover 1/3rd
% of every dimension.  Should trim off a large percentage of outliers,
% and allow the median filter to do its job better.
num_subbins_per_dim = 9;


%Figure out the minimum fraction of required points in the best window
%to accept, as a function of the number of votes.
if size(camera_relation_votes,1) < 5
    min_fraction_in_sliding_window = 1.1; %Reject less than 5 votes
elseif size(camera_relation_votes,1) < 10
    min_fraction_in_sliding_window = 0.5; %Reject less than 5 votes
elseif size(camera_relation_votes,1) < 15
    min_fraction_in_sliding_window = 0.3; %Reject less than 5 votes
else
    min_fraction_in_sliding_window = 0.1; %Reject less than 5 votes
end



%Divide up the angle range into sub-bins (we will use a sliding window
%over a 3x3x3 sub-bin range
angle_num_subbins = num_subbins_per_dim;
angle_subbin_size = (2*pi)/angle_num_subbins; 
angle_bin_starts = [0:angle_subbin_size:(2*pi-0.00001)];


%Divide up the radius. The radius can extend a really long distance, so
%lets divide up a range that includes a large percentage of points, so that
%we don't extend out too far and lose resolution to include the outliers.
%Loop over 3 parameters
fraction_of_points_to_bin_by_r = 0.75; %Only bin 75% of R, but we will devote N-1 bins to this range and go 1 farther
sorted_r = sort(camera_relation_votes(:,2), 'ascend');
%Calculate the Nth percentile radius - use ceil to round up for small
% sample sizes.
r_max = sorted_r(ceil(length(sorted_r)*fraction_of_points_to_bin_by_r));
r_num_subbins = num_subbins_per_dim;
r_subbin_size = r_max/(r_num_subbins-1); %do N-1 bins on the range 0 to r_max
r_bin_starts = [0:r_subbin_size:(r_max+r_subbin_size - 0.000001)]; % and go 1 farther

%Put votes in appropriate bins
vote_hist = zeros(angle_num_subbins, r_num_subbins, angle_num_subbins);
for i=1:length(camera_relation_votes)
    vote = camera_relation_votes(i,:);
    vote(1) = mod(vote(1), 2*pi);
    vote(3) = mod(vote(3), 2*pi);
    if vote(2) < r_max
        vote_bin(1) = floor(vote(1) / angle_subbin_size) + 1;
        vote_bin(3) = floor(vote(3) / angle_subbin_size) + 1;
        vote_bin(2) = floor(vote(2) / r_subbin_size) + 1;
        
        vote_hist(vote_bin(1), vote_bin(2), vote_bin(3)) ...
            = vote_hist(vote_bin(1), vote_bin(2), vote_bin(3)) + 1;
    end
end

%Apply window - basically convolve with a 3x3x3 array of 1's to get
%3x3x3 sliding window bins. Edge cases are different for angles and
%distances. Angles wrap, since periodic. Distances do not.
window_hist = zeros(angle_num_subbins, r_num_subbins, angle_num_subbins);
for i = 1:size(vote_hist,1)
    ibins = [i-1, i, i+1]; %Select 3 wide window around i
    ibins(ibins<1) = size(vote_hist,1); %Wrap left edge, since angle is periodic
    ibins(ibins>size(vote_hist,1)) = 1; %Wrap right edge, since angle is periodic
    for j = 1:size(vote_hist,2)
        jbins = [j-1, j, j+1]; %Select 3 wide window around j
        jbins(jbins<1) = [];                 %Clip left end wrapping, since distance is not periodic
        jbins(jbins>size(vote_hist,2)) = []; %Clip right end wrapping, since distance is not periodic
        for k = 1:size(vote_hist,3)
            kbins = [k-1, k, k+1]; %Select 3 wide window around i
            kbins(kbins<1) = size(vote_hist,3); %Wrap left edge, since angle is periodic
            kbins(kbins>size(vote_hist,3)) = 1; %Wrap right edge, since angle is periodic
            
            window_hist(i,j,k) = sum(sum(sum(vote_hist(ibins,jbins,kbins))));
        end
    end
end

[s_binvotes, s_bininds] = sort(window_hist(:), 'descend');

% fprintf('Ratio %f, num_votes=%.0f, num_votes total = %.0f\n', ...
%             s_binvotes(1) / length(camera_relation_votes), ...
%             s_binvotes(1), length(camera_relation_votes));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%If not enough votes in best window, return empty
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if s_binvotes(1) < length(camera_relation_votes) * min_fraction_in_sliding_window
    center = [];
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%If enough votes in best window, first find all the votes in the window
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[x,y,z] = ind2sub(size(window_hist), s_bininds(1));
bin_inds = [x y z]
%Loop over dims, find the start and end indices of the window in the
%histogram
for i=1:3
    bin_starts(i) = bin_inds(i)-1;
    %Fix left end underflow
    if bin_starts(i)<1
        if i == 1 || i == 3 %angular dimensions are periodic so wrap left side
            bin_starts(i) = size(window_hist,i);
        else %radius is not periodic so clip
            bin_starts(i) = 1;
        end
    end
    bin_ends(i) = bin_inds(i)+1;
    %Fix right end overflow
    if bin_ends(i)>size(window_hist,i)
        if i == 1 || i == 3 %angular dimensions are periodic so wrap left side
            bin_ends(i) = 1;
        else %radius is not periodic so clip
            bin_ends(i) = size(window_hist,i);
        end
    end
end

%Find all the votes in the window
votes_in_window = [];
for v=1:length(camera_relation_votes)
    vote = camera_relation_votes(v,:);
    vote(1) = mod(vote(1), 2*pi);
    vote(3) = mod(vote(3), 2*pi);
    if vote(2) < r_max
        vote_bin(1) = floor(vote(1) / angle_subbin_size) + 1;
        vote_bin(3) = floor(vote(3) / angle_subbin_size) + 1;
        vote_bin(2) = floor(vote(2) / r_subbin_size) + 1;

        %Check if in window in every dim
        satisfied_dims = 0;
        for d=1:3
            if bin_starts(d) < bin_ends(d) %Window doesn't wrap
                if((bin_starts(d) <= vote_bin(d)) & (vote_bin(d) <= bin_ends(d)))
                    satisfied_dims = satisfied_dims + 1;
                end
            else % Window does wrap - so make sure vote isn't between the boundaries
                if~((bin_ends(d) < vote_bin(d)) & (vote_bin(d) < bin_starts(d)))
                    satisfied_dims = satisfied_dims + 1;
                end
            end
        end
        if satisfied_dims == 3
            votes_in_window = [votes_in_window v];
        end
    end
end

%Sanity check - should get the same number of votes here as we did earlier
if length(votes_in_window) ~= s_binvotes(1)
    keyboard
    error('Voting is screwing up while finding the votes in the peak window.');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Set the center to the median in each dimension of all the votes in the
%window. Need to handle the angles specially because they are periodic,
%so if we are crossing an edge from +pi -> -pi, need to make all the angles
%into their version near +pi.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

relevent_votes = camera_relation_votes(votes_in_window,:);

%First need to adjust angles if some are near -pi and some near +pi
for dim=[1,3]
    %Only offset votes if they span a boarder, otherwise could cause
    %problems
    if any(relevent_votes(:,dim) > pi) ...
    && any(relevent_votes(:,dim) < -pi)
        %Move the negative votes to be by +pi by adding 2*pi
        relevent_votes( relevent_votes(:,dim) < 0) ...
            = relevent_votes( relevent_votes(:,dim) < -pi) + 2*pi;
    end
end

%Do the centering step, using the median
center = median(camera_relation_votes(votes_in_window,:),1);

%Bring back angles to the range -pi to pi, if offsetting pushed them higher
if center(1) > pi
    center(1) = center(1) - 2*pi;
end
if center(3) > pi
    center(3) = center(3) - 2*pi;
end

end
