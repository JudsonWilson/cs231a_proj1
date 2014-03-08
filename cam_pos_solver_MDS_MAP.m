function [ solved_cam_positions ] = cam_pos_solver_mds_map(num_cameras, camera_distance_estimates )
%CAM_POS_SOLVER_MDS_MAP Estimate camera positions using MDS-MAP.
% Inputs:
%   - num_cameras - The number of cameras.
%   - camera_distances - A list of distance constraints between cameras,
%         as rows in the format [cam_num_1, cam_num_2, distance]
%         Does NOT need to be fully connected.
% Outputs:
%   - solved_cam_positions - 2d list of points, where rows are points.
%
% Uses Judson's MDS-MAP routine.
%

N = num_cameras;
CDEs = camera_distance_estimates;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Calculate distance matrix D from camera contraints
% - note the MDS_MAP routine will make the matrix symmetric, we don't
%   need to do this here.
D = inf*ones(N);
for i=1:size(CDEs,1)
    D(CDEs(i,1), CDEs(i,2)) = CDEs(i,3);
end
%We know the diagonal is zero.
D(logical(eye(N))) = 0;

%%%%%%%%%%%%%%%
%Do the MDS-MAP
solved_cam_positions = mds_map(D);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Force the result to be real
if ~isreal(solved_cam_positions)
    fprintf(['Error: the MDS-MAP result is not real-valued, forcing ' ...
            'it to be real, but it won''t be pretty.\n']);
    solved_cam_positions = real(solved_cam_positions);
end
end

