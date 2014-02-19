function [ individual_tracklets_cam_coords, correspondences, tracks_with_tracklets_world_coords ] ...
  = generate_tracklets_advanced( cameras, num_correspondences, objects_start_speed, ...
       shape_noise_factor, measurement_noise_factor, ... 
       track_coincidence_percentiles, fraction_singles_not_pairs)
%GENERATE_TRACKLETS_ADVANCED Walk objects somewhat randomly in tracks.
% Try and generate tracklets where there are multiple tracks happening
% simultaneouly (thus false correspondences) with the specified frequency
% for each amount of simultaneous tracks (i.e. 1 track, 2 tracks, ...)
%
% individual_tracklets - a cell array of all tracklet structures.
%
% correspondences - a cell array of correspondence indice arrays. Each
%     cell contains an array of indices from individual_tracklets. 
%
% tracks_with_tracklets - Intended for static plotting of all the paths.
%
%
% In the future - make this in actual data format, and let the algorithm
% discern the correspondences from time.

bound_box = cameras_bound_box(cameras);

%Where the data goes that gets returned by this function
tracklets_cam_coords_final = {};
tracks_with_tracklets_world_coords_final = {};
correspondences_final = {};

%Temporary pools of generated tracks to draw from to make
% correspondences.
tracks_with_tracklets_pool_singles = {};
tracks_with_tracklets_pool_pairs = {};

%Variables for calculating average track time, for use in figuring out
% how to space tracks in time when making false correspondances.
average_track_time_sum = 0;
average_track_time_count = 0;
average_track_time = 0;

%
% Make the specified number of tracks. Only count tracks that lead to
% tracklets (some tracks are outside all camera FOVs, ignore them).
%
tracks_made = 0;
while size(correspondences_final) < num_correspondences
    % Randomly pick the number of tracks that we want at the same time,
    % according to our global settings above: 
    %      track_coincidence_percentiles
    % Will be something like 1,2,3...
    num_tracks_in_time_window ...
         = find(rand(1) < track_coincidence_percentiles, 1);
%    % Only use as many tracks as are requested
%    tracks_remaining = num_tracks - tracks_made;
%    num_tracks_in_time_window = min(num_tracks_in_time_window, ...
%                                    tracks_remaining);

    %Choose which of the tracks in the time window come from single
    %tracklets (viewed from only one camera), and those from pairs
    if num_tracks_in_time_window > 1
        single_tracks_logical = rand(num_tracks_in_time_window,1) < fraction_singles_not_pairs;
    else
        single_tracks_logical = 0; %one track with multiple tracklets.
    end
    %How many do we need?
    num_singles_needed = sum(single_tracks_logical);
    num_pairs_needed = num_tracks_in_time_window - num_singles_needed';
    
    %Get more tracks with tracklets to fill up the pools of singles / pairs
    % if we know there aren't enough for this job.
    while length(tracks_with_tracklets_pool_singles) < num_singles_needed || ...
          length(tracks_with_tracklets_pool_pairs) < num_pairs_needed
        % Get a track with tracklets. 
        new_track_with_tracklets = generate_track_with_tracklets( cameras, objects_start_speed, shape_noise_factor, measurement_noise_factor);
        % We don't know how many tracklets there will be, since it's
        % random, so figure that out afterwards. Only add to pool if the
        % pool doesn't have too many already
        if length(new_track_with_tracklets.tracklets) < 2
            if length(tracks_with_tracklets_pool_singles) < 20
                tracks_with_tracklets_pool_singles(end + 1) = {new_track_with_tracklets};
            end
        else
            if length(tracks_with_tracklets_pool_pairs) < 20
                tracks_with_tracklets_pool_pairs(end + 1) = {new_track_with_tracklets};
            end
            
            %If we have multiple tracklets, add this to the average track
            %time statistics.
            [~,~,track_time] = trackblob_get_timeinfo( new_track_with_tracklets );
            average_track_time_sum = average_track_time_sum  + track_time;
            average_track_time_count = average_track_time_count + 1;
            average_track_time = average_track_time_sum ...
                                 / average_track_time_count;
        end
    end
    
    %Generate the correspondence. Basic idea: make each track start at
    % time 0. Then figure out a random amount of sensical offset such
    % that we get a random (false) correspondence.

    correspondance_list_array = [];
    
    for track_num_in_correspondence=1:num_tracks_in_time_window
        %Take a track from the single or pairs pool.
        if single_tracks_logical(track_num_in_correspondence) == 1
            temp_track = tracks_with_tracklets_pool_singles{1};
            tracks_with_tracklets_pool_singles(1) = [];
        else
            temp_track = tracks_with_tracklets_pool_pairs{1};
            tracks_with_tracklets_pool_pairs(1) = [];
        end

        %Figure out time offset for this track.

        % Offset the 2nd, 3rd, etc from eachother in time
        if track_num_in_correspondence == 1
            cumulative_offset = 0;
        else
            new_offset = (1.5 + randn(1))*average_track_time; %1.5 chosen a bit arbitrarily. 
            cumulative_offset = cumulative_offset + new_offset;
        end
        %Remove offset of first tracklet in each track such that it starts
        % at time 0, then apply cumulative offset for 2nd, 3rd, etc tracks.
        [min_t,~,~] = trackblob_get_timeinfo(temp_track);
        offset = -min_t + cumulative_offset;

        %Operate on each tracklet in the track blob
        for j=1:length(temp_track.tracklets)
            % Set time to 0 for first tracklet and offset others the same.
            temp_track.tracklets{j}.first_time = temp_track.tracklets{j}.first_time + offset;
            % Add tracklets to lists and the correspondence_array
            tracklets_list_pos = length(tracklets_cam_coords_final) + 1;
            correspondance_list_array(end + 1) = tracklets_list_pos;
            tracklets_cam_coords_final{tracklets_list_pos} = transform_tracklet_world_to_camera(cameras, temp_track.tracklets{j});
        end
        %Add this track to the output list
        tracks_with_tracklets_world_coords_final{end+1} = temp_track;
    end

    correspondences_final{end+1} = correspondance_list_array;
end

%return values
individual_tracklets_cam_coords  = tracklets_cam_coords_final;
tracks_with_tracklets_world_coords = tracks_with_tracklets_world_coords_final;
correspondences = correspondences_final;