function [individual_tracklets_cam_coords] = load_external_data(filename)

    % Read In Tracklet Array
    tracklet_array = csvread(filename);
    [m,n] = size(tracklet_array);
    num_tracklets = floor(n/3);
    
    % Make Tracklet Objects
    for i = 1:num_tracklets
        % Get Camera Number
        tmp_tracklet.cam_num = tracklet_array(1:3*(i-1)+1);
        
        % Get Length of Tracklet
        len_valid_tracklet = tracklet_array(1:3*(i-1)+2);
        
        % Get First, Last Time
        tmp_tracklet.first_time = tracklet_array(2:3*(i-1)+3);
        tmp_tracklet.last_time = tracklet_array(len_valid_tracklet+1, ...
                                                3*(i-1)+3);
        
        % Get Tracklet Path
        tmp_tracklet.path = tracklet_array(2:len_valid_tracklet+1, ...
                                           3*(i-1)+1:3*(i-1)+2);
        
        % Add Tracklet to List
        individual_tracklets_cam_coords{i+1} = tmp_tracklet;
    end
    
    % Generate Correspondences
end