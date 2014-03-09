function [individual_tracklets_cam_coords, correspondences,dt] ...
            = load_external_data(filename, ...
                                 corr_window_len)

    % Read In Tracklet Array
    tracklets_array = dlmread(filename,' ');
    [m,n] = size(tracklets_array);
    num_tracklets = m;
    data_len = (n-1)/3;
    correspondences = [];
%     tracklets_array(:,1:3)
    
    % Intitialize Timing
    dt = inf;
    t0 = inf;
    tf = 0;
    
    disp('Making Tracklet Objects')
    
    % Make Tracklet Objects
    for i = 1:num_tracklets
        % Reshape Tracklet
        tmp_tracklet_array = reshape(tracklets_array(i,1:end-1),3,data_len)';
        
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
        tmp_tracklet.path = tmp_tracklet_array(2:len_valid_tracklet+1, 1:3);
        
        % Get Min dt
        tmp_dt_min = min(tmp_tracklet.path(2:end,3) - tmp_tracklet.path(1:end-1,3));
        
        % Update dt
        if (tmp_dt_min < dt)
            dt = tmp_dt_min;
        end
        
        % Add Tracklet to List
        individual_tracklets_cam_coords{i} = tmp_tracklet;
    end
    
    % Generate Correspondences
    % Create time array
    t = t0:dt:tf;
    bins = cell(length(t),1);
    
    disp('Binning Tracklets')
    
    % Bin Tracklets by time
    for i = 1:num_tracklets
        % Find the Bins this tracklet goes in
        tracklet_t0 = individual_tracklets_cam_coords{i}.first_time;
        tracklet_tf = individual_tracklets_cam_coords{i}.last_time;
        start_bin = find((t <= tracklet_t0),1,'last');
        end_bin = find((t <= tracklet_tf),1,'last');
        
        % Add Tracklet Index and camera to each bins in its range
        tracklet_cam_num = individual_tracklets_cam_coords{i}.cam_num;
        for bin_ind = start_bin:end_bin
            bins{bin_ind} = [bins{bin_ind}, [i;tracklet_cam_num]];
        end
    end
    
    disp('Making Correspondences')
    correspondence_table = zeros(num_tracklets);
    tmp = [];
    % Come up with correspondences by window length
    for i = 1:length(t)-corr_window_len
        % Get Bins correspoinding to this window
        bins_arr = [bins{i:i+corr_window_len-1}];
        
        % Skip Empty Windows
        if (isempty(bins_arr))
            continue
        end
        
%         tmp = [tmp;size(bins_arr)];
%         disp(sprintf('Ind: %d',i))
%         unique(bins_arr(1,:))
        % Check if multiple objects occur in the same camera in this window
        n = histc(bins_arr(2,:),unique(bins_arr(2,:)));
        if (sum(n>1) > 0)
            % Multiple objects in a single camera in this window
            %***************DO SOMETHING HERE********************
        end
        
        % Make Correspondences
        unique_tracklets = unique(bins_arr(1,:));
        for j = 1:length(unique_tracklets)
            for k = j+1:length(unique_tracklets)
                % Make sure unique correspondence and different camera
                if ((~correspondence_table(unique_tracklets(j),unique_tracklets(k))) & ...
                    (individual_tracklets_cam_coords{unique_tracklets(j)}.cam_num ~= ...
                     individual_tracklets_cam_coords{unique_tracklets(k)}.cam_num))
                    correspondences = [correspondences;unique_tracklets([j,k])];
                    correspondence_table(unique_tracklets(j),unique_tracklets(k)) = 1;
                end
            end
        end
    end
    
%     tmp
    disp('Completed Correspondences')
end