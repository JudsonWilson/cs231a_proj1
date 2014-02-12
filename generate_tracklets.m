function [ tracklets, tracks_with_tracklets ] = generate_tracklets( cameras, num_tracks, objects_start_speed, shape_noise_factor, measurement_noise_factor)
%GENERATE_TRACKLETS Walk objects somewhat randomly in tracks.
% Return the tracklets (an array per camera), as well as tracks
% with the resulting tracklets, for plotting purposes.

bound_box = cameras_bound_box(cameras);

tracklets = {};
tracks_with_tracklets = {};

%
% Make the specified number of tracks. Only count tracks that lead to
% tracklets (some tracks are outside all camera FOVs, ignore them).
%
tracks_made = 0;
while tracks_made < num_tracks
    %Get a new track.
    track = generate_track(bound_box, objects_start_speed, shape_noise_factor, measurement_noise_factor);
    track_tracklets = {};
    %Get all tracklets for each camera
    for c=1:length(cameras)
        cam = cameras(c);
        track_index = 1; %start at first point in track
        last_point_in_fov = false; %Forces new tracklet to start
        %Start at first point on track, and march till the end.
        while track_index <= size(track,1)
            %See if this point is in the FOV
            this_point_in_fov = camera_can_see(cam, track(track_index,1), ...
                                                    track(track_index,2));
            %If point is in FOV, either update tracklet, or start a new
            %one.
            if this_point_in_fov
                %Start new tracklet?
                if last_point_in_fov == false
                    clear cam_tracklet_inst;
                    cam_tracklet_inst.cam_num = c;
                    cam_tracklet_inst.path = zeros(0,2);
                    %For now, set the start time to the index in the track
                    % relative time (e.g. starting at 0 for each track)
                    % since for now we only consider correspondences
                    % from the same track and the tracks never overlap.
                    cam_tracklet_inst.first_time = track_index;
                end
                %Add the data to the tracklet.
                cam_tracklet_inst.path(end+1,:) = track(track_index,:);
            %If point is not in FOV, see if it is time to end the tracklet.
            else
                %Is this the end?
                if last_point_in_fov
                    %Add the tracklet to the list of them for this track
                    track_tracklets{end+1} = cam_tracklet_inst;
                end
            end
            
            last_point_in_fov = this_point_in_fov;
            track_index = track_index + 1;
        end
    end

    %If we hit end of FOV while still in tracklet, we want to keep this
    % tracklet (never got to the end of it).
    if last_point_in_fov
        %Add the tracklet to the list of them for this track
        track_tracklets{end+1} = cam_tracklet_inst;
    end

    
    
    
    %Only add tracklets if they exist, and only update the number of
    %tracks made if the track actually lead to tracklets (i.e. intersected
    %the FOV of at least one camera).
    if length(track_tracklets) > 0
        %A new group track and its tracklets for the mother array
        track_with_tracklet.track = track;
        track_with_tracklet.tracklets = track_tracklets;

        %Update the mother array
        tracks_with_tracklets{end+1} = track_with_tracklet;
        tracklets = [tracklets track_tracklets{:}];

        %Remove variables to reduce confusion
        clear track_with_tracklet;
        
        tracks_made = tracks_made + 1;
    end
end

end

