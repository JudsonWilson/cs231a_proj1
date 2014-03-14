function [ correspondences, ground_truth ] ...
       = generate_fake_test_data(cameras,...
                                 number_of_correspondences, ...
                                 track_velocity_factor, ...
                                 track_non_constant_factor, ...
                                 track_observation_variance_scale, ...
                                 multiple_objects_collision_percentiles, ...
                                 multiple_correspondence_percentage_single_tracklets, ...
                                 plot_yes_no)
%
%
% Outputs:
%     - correspondencs - A structure of tracklets (tracks as seen in
%           a camera, so if a track goes through multiple cameras it gets
%           chopped into 1 tracklet per camera), and correspondences, which
%           are a (tr1,tr2) pair saying which tracklets likely correspond.
%           Basic rundown of fields:
%               .num_cameras - The number of cameras.
%               .tracklets_cam_coords   - A cell array of structures
%                                         representing a tracklet in camera
%                                         coordinates. Fields defined
%                                         below.
%               .tracklets_cam_coords(i).cam_num - Number of the camera
%                                                  that the tracklet is in.
%               .tracklets_cam_coords(i).path    - (n x 3) list of
%                                                  coordinates and times
%                                                  in format [x y t]
%               .tracklets_cam_coords(i).first_time - time of first point.
%                                                  This time scheme may
%                                                  need to change in the
%                                                  future. Right now you
%                                                  just assume each point
%                                                  is a time step.
%               .tracklet_pairings - An n x 2 list of pairings,
%                                    identifying tracklet array indices.
%               
%     - ground_truth - An array of camera structures, with at least the
%           following fields:
%             .cameras(i).calib.x      - x position in world
%             .cameras(i).calib.y      - x position in world
%             .cameras(i).calib.theta  - angle of rotation in 2D world
%                                        in radians
%   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Cameras and Bound Box
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if plot_yes_no
    hold on;

    % Show the two cameras
    for c = 1:length(cameras)
        plot_poly(cameras(c).gen.fov_poly_world);
    end
    axis equal;

    %Plot a bound-box
    bound_box = cameras_bound_box(cameras);
    plot( [bound_box.x(1); bound_box.x(2); bound_box.x(2); bound_box.x(1); bound_box.x(1)], ...
          [bound_box.y(1); bound_box.y(1); bound_box.y(2); bound_box.y(2); bound_box.y(1)], ...
          'r');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get Tracklets and Tracks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[ tracklets_cam_coords, correspondences_lists, tracks_with_tracklets_world_coords ] ...
    = generate_tracklets_advanced( cameras, number_of_correspondences, ...
       track_velocity_factor, track_non_constant_factor, ...
       track_observation_variance_scale, ...
       multiple_objects_collision_percentiles, ...
       multiple_correspondence_percentage_single_tracklets );

%The correspondences_lists are not pairwise, they are n-wise. Figure out
%all the pairings.
correspondences_pairs = [];
for i=1:length(correspondences_lists)
    %Correspondences for this time window
    correspondences_list_now = correspondences_lists{i};
    %We will do every pairing of tracklets in this correspondence list,
    %as long as they are from different cameras.
    correspondences_pairs = [correspondences_pairs, 
                             nchoosek(correspondences_list_now, 2)];
end    

   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot tracks and tracklets
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if plot_yes_no
    for i=1:length(tracks_with_tracklets_world_coords)
        track_with_tracklets = tracks_with_tracklets_world_coords{i};
        track = track_with_tracklets.track;
        tracklets = track_with_tracklets.tracklets;
        %Plot track
        plot(track(:,1),track(:,2),'-g');
        plot(track(:,1),track(:,2),'.b');
        %Plot starting point
        plot(track(1,1),track(1,2),'ob');
        %Plot tracklets associated with track
        for j=1:length(tracklets)
            tracklet = tracklets{j};
            %Plot tracklet
            plot(tracklet.path(:,1),tracklet.path(:,2),'-r');
            plot(tracklet.path(:,1),tracklet.path(:,2),'.m');
            %Plot tracklet starting point
            plot(tracklet.path(1,1),tracklet.path(1,2),'om');
        end
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate return values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

correspondences.num_cameras           = length(cameras);
correspondences.tracklets_cam_coords  = tracklets_cam_coords;
correspondences.tracklet_pairings     = correspondences_pairs;
ground_truth.cameras = cameras;
