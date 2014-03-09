function [ solved_cam_positions ] = cam_pos_solver_SDP1_3plusanchors...
                                       (num_cameras, ...
                                        camera_distance_estimates,...
                                        anchors_numbers, anchor_locations)
%CAM_POS_SOLVER_SDP1_3PLUSANCHORS Estimate camera positions using
% Semidefinate Programming technique. The technique requires at least
% three anchors, so this version accepts 3 or more anchors.
%
% Technique/formulation of the SDP approach is as described in the
% paper "Second-Order Cone Programming Relaxation of Sensor Network
% Localization" by Paul Tseng. It is a slight change of notation
% from the original version in "Semidefinite Programming for Ad Hoc
% Wireless Sensor Network Localization" by Pratik Biswas and Yinyu Ye.
%
% Inputs:
%   - num_cameras - The number of cameras.
%   - camera_distance_estimates - A list of distance constraints between
%         cameras, as rows in the format [cam_num_1, cam_num_2, distance]
%         Does NOT need to be fully connected.
%   - anchors_numbers - List of camera numbers to use as anchors.
%   - anchor_locations - locations of the cameras to use as anchors.
% Outputs:
%   - solved_cam_positions - 2d list of points, where rows are points.
%
% Uses CVX, a convex optimization package, by CVXR, to solve the SDP.
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Check anchors input parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ndims(anchors_numbers) ~= 1 && ...
    ~(ndims(anchors_numbers) == 2 && (size(anchors_numbers,1) == 1 || ...
                                      size(anchors_numbers,2) == 1 ))
    error('anchors_numbers must be a 1d array!');
end    
if length(anchors_numbers) < 3
    error('Require at least 3 anchors!');
end    
num_anchors = length(anchors_numbers);
if num_anchors ~= size(anchor_locations,1)
    error('num_anchors does not match the number of anchor_positions');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The method requires arranging all points (cameras) such that the first
% ones in the list are variable position, and the remaining ones at the
% end of the list are fixed anchors.  Here we calculate the mapping of
% the camera indexes from their native "old" list indexes to their
% "new" indexes for this algorithm, and the reverse mapping, to shuffle
% the coordinate lists around before/after the algorithm is run

%Find mapping to/from an ordering where anchors are last,
% in the terminonlogy of the paper, we want [x x x x x a a a]
anchors_logical = zeros(1,num_cameras);
anchors_logical(anchors_numbers) = 1;
non_anchors_logical = ~anchors_logical;

%original to new ([x1 x2 a1 x3 a2 x4 x5] -> [x1 x2 x3 x4 x5 a1 a2])
for i = 1:num_cameras
    %If it's an anchor, its somewhere after the non-anchors, specifically,
    % if it's the nth anchor then it's n positions later.
    if anchors_logical(i)
        o2n(i) = sum(non_anchors_logical) + sum(anchors_logical(1:i));
    %If it's the nth non-anchor, then its n positions after 0
    else
        o2n(i) = sum(non_anchors_logical(1:i));
    end
end
%new to original ([x1 x2 x3 x4 x5 a1 a2] -> [x1 x2 a1 x3 a2 x4 x5])
for i = 1:num_cameras
    for j=1:num_cameras
        if o2n(j) == i
            n2o(i) = j;
        end
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup d and m constants, as described in the paper
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%2 dimensional
d = 2; 
%Number of cameras
n = num_cameras;
%Number of non-anchored cameras
m = n - num_anchors; %We anchor camera 4 at 0,0

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup the bij constants as described in the paper.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b={};
for i=1:size(camera_constraints,1)
    ei = zeros(num_cameras,1);
    ei(o2n(camera_constraints(i,1))) = 1;
    ej = zeros(num_cameras,1);
    ej(o2n(camera_constraints(i,2))) = 1;
    b{i} = [      eye(m),  zeros(m,num_anchors);  ...
             zeros(2, m),     anchor_locations' ] ...
           *(ei-ej); %Assume 1 anchor at 0,0
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup and solve the convex SDP problem, as described in the paper
% - The following is a cvx convex optimization problem, for the cvx
%   package, available at http://www.cvxr.com/cvx
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cvx_begin quiet %sdp
    variable u(size(camera_constraints,1))
    variable v(size(camera_constraints,1))
    variable Z(m+d,m+d) symmetric
    
    minimize sum(u) + sum(v)
    subject to
        for i=1:size(camera_constraints,1)
            sum(sum((b{i}*b{i}').*Z')) - u(i) + v(i) == camera_constraints(i,3)^2
        end
        Z((m+1):end,(m+1):end) == eye(d)
        u >= 0
        v >= 0
        Z == semidefinite(m+d)
cvx_end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Fill in the solved camera positions,
% Note that here we re-ordering back to the original ordering where the
% anchors are at various positions in the list of camera locations. The
% optimization result has the unwanted ordering where the anchors are last.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
solved_cam_positions = zeros(num_cameras,2);
solved_cam_positions(anchors_numbers,:) = anchor_locations;
solved_cam_positions(non_anchors_logical,:) = Z(1:m,(m+1):end);

if ~isreal(solved_cam_positions)
    disp(solved_cam_positions);
    error('unreal cam_pos_solver_SDP1_3plusanchors result!');
end


end

