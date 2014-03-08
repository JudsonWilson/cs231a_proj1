function [individual_tracklets_cam_coords,correspondences] ...
            = load_external_data(filename, ...
                                 corr_window_len)

    % Read In Tracklet Array
    tracklets_array = csvread(filename);
    [m,n] = size(tracklets_array);
    num_tracklets = m;
    data_len = n/3;
    
    % Intitialize Timing
    dt = inf;
    t0 = inf;
    tf = 0;
    
    % Make Tracklet Objects
    for i = 1:num_tracklets
        % Reshape Tracklet
        tmp_tracklet_array = reshape(tracklets_array(i,:),3,data_len)';
        
        % Get Camera Number
        tmp_tracklet.cam_num = tmp_tracklet_array(1,1);
        
        % Get Tracklet ID Num (NOTE: this is unique to this camera only)
        tmp_tracklet.id_num = tmp_tracklet_array(1,2);
        
        % Get Length of Tracklet
        len_valid_tracklet = tmp_tracklet_array(1,3);
        
        % Get First, Last Time
        tmp_tracklet.first_time = tmp_tracklet_array(2,3);
        tmp_tracklet.last_time = tmp_tracklet_array(len_valid_tracklet+1,3);
                    
        % Update t0,tf
        if (t0 > tmp_tracklet.first_time)
            t0 = tmp_tracklet.first_time;
        end
        if (tf < tmp_tracklet.last_time)
            tf = tmp_tracklet.last_time;
        end
        
        % Get Tracklet Path (n-by-3 array)
        tmp_tracklet.path = tracklet_array(2:len_valid_tracklet+1, ...
                                           3*(i-1)+1:3*(i-1)+3);
        
        % Get Min dt
        tmp_dt_min = min(tmp_tracklet.path(2:end,3) - tmp_tracklet.path(1:end-1,3));
        
        % Update dt
        if (tmp_dt_min < dt)
            dt = tmp_dt_min;
        end
        
        % Add Tracklet to List
        individual_tracklets_cam_coords{i+1} = tmp_tracklet;
    end
    
    % Generate Correspondences
    % Create time array
    t = t0:dt:tf;
    bins = cell(length(t),1);
    
    % Bin Tracklets by time
    for i = 1:num_tracklets
        % Find the Bins this tracklet goes in
        tracklet_t0 = individual_tracklets_cam_coords{i}.first_time;
        tracklet_tf = individual_tracklets_cam_coords{i}.last_time;
        start_bin = find((~(tracklet_t0 < t)) & (~(tracklet_t0 > t)),1,'first');
        end_bin = find((~(tracklet_tf < t)) & (~(tracklet_tf > t)),1,'last');
        
        % Add Tracklet Index and camera to each bins in its range
        tracklet_cam_num = individual_tracklets_cam_coords{i}.cam_num;
        for bin_ind = start_bin:end_bin
            bins{i} = [bins{i}, [i;tracklet_cam_num]];
        end
    end
    
    % Come up with correspondences by window length
    for i = 1:length(t)-corr_window_len
        % Get Bins correspoinding to this window
        bins_arr = [bins{i:i+corr_window_len-1}];
        
        % Check if multiple objects occur in the same camera in this window
        n = histc(bins_arr(2,:),unique(bins_arr(2,:)));
        if (sum(n>1) > 0)
            % Multiple objects in a single camera in this window
            %***************DO SOMETHING HERE********************
        end
        
        % Make Correspondences
        unique_tracklets = unique(bins_arr(1,:));
        for j = 1:length(unique_tracklets)
            for k = j:length(unique_tracklets)
                correspondences = [correspondences;unique_tracklets([j,k])];
            end
        end
        
    end
    
    
    
end