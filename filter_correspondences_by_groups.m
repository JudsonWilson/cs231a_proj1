function [new_pairings,shapes] = filter_correspondences_by_groups(correspondences, ...
                                         time_scaling_factor, ...
                                         match_cutoff)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creates a shape descriptor for each tracklet, and uses this    %
% desciptor to remove correspondences that do not have matching  %
% shapes.                                                        %
%                                                                %
% the correspondences structure will be modified by removing     %
% entries from the .traklet_pairings field                       %
%                                                                %
% Parameters:                                                    %
%   -correspondences: output from load_external_data.m           %
%   -time_scaling_factor: scalar value to scale the time value   %
%       for the KD-tree lookup to make it equal weight to        %
%       distances                                                %
%   -bins: array with the radial bin distance for the histogram  %
%       that will define the group shape                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  bins = zeros(4,1);
  radius = 40;

  max_cam_num = 0;
  active_cameras = [];
  for i=1:length(correspondences.tracklets_cam_coords)
    cam_num = correspondences.tracklets_cam_coords{i}.cam_num;
    if(cam_num > max_cam_num)
      max_cam_num = cam_num;
    end
    if(~ismember(cam_num, active_cameras))
      active_cameras = [active_cameras cam_num];
    end
  end

  num_tracklets = length(correspondences.tracklets_cam_coords);

  % create array of every point for each camera
  %camera_points = cell(correspondences.num_cameras);
  camera_points = cell(max_cam_num);
  for i=1:max_cam_num
    camera_points{i} = [];
  end

  % populate camera_points array
  for i=1:num_tracklets
    tracklet = correspondences.tracklets_cam_coords{i};
    cam_num = active_cameras(tracklet.cam_num);
    % append the tracklet index to each point, will be needed to get the tracklet
    % number from a kdtree search result
    x_coords = tracklet.path(:,1);
    y_coords = tracklet.path(:,2);
    t_coords = tracklet.path(:,3)/time_scaling_factor;
    trk_id  = i*ones(size(tracklet.path,1),1);
    %points = [tracklet.path ones(size(tracklet.path,1),1)*i];
    points = [x_coords y_coords t_coords trk_id];
    camera_points{cam_num} = [camera_points{cam_num}; points];
  end

  % build a KD-tree for each camera
  kdtrees = cell(max_cam_num);
  for i=1:size(active_cameras,2)
    cam = active_cameras(i);
    % ignore the tracklet index field in the KD tree
    kdtrees{cam} = KDTreeSearcher(camera_points{cam}(:,1:3), 'distance', 'euclidean');
  end

  % create and initialize the shape histogram for each tracklet
  %shapes = cell(num_tracklets);
  shapes = zeros(num_tracklets,size(bins,1));


  disp('Creating group shapes for tracklets');
  % create shape for each tracklet
  for trackletId=1:num_tracklets
    tracklet = correspondences.tracklets_cam_coords{trackletId};
    cam_num = active_cameras(tracklet.cam_num);
      
    accumulatedHistogram = zeros(size(bins,1),1);

    % create shape for each point on tracklet and take average of all points'
    % shapes to get the shape for the tracklet
    for i=1:size(tracklet.path,1)
      curPoint = tracklet.path(i,:);
      curPoint(3) = curPoint(3)/time_scaling_factor;

      curHistogram = zeros(size(bins,1),1);

      matches = [];
      cur_bin_count = -1;
      indexes = rangesearch(kdtrees{cam_num}, curPoint, radius);
      inds = indexes{1}; 

      % for each result contained in indexes, push it into the
      % matches array to keep track of which tracklets we 
      % have found a result from
      for l=1:size(inds,2)
        match = camera_points{cam_num}(inds(l),:);
        found = 0;
        % check if the tracklet the result belongs to has already
        % been counted in a previous result
        for k=1:size(matches,1)
          if(matches(k,4) == match(4))
            found = 1;
            break;
          end
        end % for k=1:size(matches,1)

        % if from a new tracklet, push to matches and increase
        % bin count for this radius
        if(found == 0)
          matches = [matches; match];
        end

      end % for l=1:size(indexes{1},1)

      % the closest point is always the current point...
      matches = matches(2:end,:);

      for k=1:min(size(matches,1),size(bins,1))
        %curPoint
        %matches(k,1:3)
        dist = norm(curPoint - matches(k,1:3));
        dist = dist/10 - 1.5;
        if(dist < 0)
          dist = 0;
        end
        weight = exp(-dist*dist/2);
        curHistogram(k) = weight;
      end
      accumulatedHistogram = accumulatedHistogram + curHistogram;
    end
    
    % the shape of the whole tracklet is the average of the shape of each
    % point in the tracklet
    accumulatedHistogram = accumulatedHistogram/size(tracklet.path,1);
    shapes(trackletId,:) = accumulatedHistogram;
  end

  shapes;

  new_pairings = [];

  disp('filtering correspondences by group shapes');

  for i=1:length(correspondences.tracklet_pairings)
    tracklet_id_1 = correspondences.tracklet_pairings(i,1);
    tracklet_id_2 = correspondences.tracklet_pairings(i,2);
    shape_1 = shapes(tracklet_id_1,:);
    shape_2 = shapes(tracklet_id_2,:);
    distance = norm(shape_1 - shape_2);
    if(norm(shape_1 - shape_2) < match_cutoff && norm(shape_1,1) > .5 && norm(shape_2,1) > 0.5)
      new_pairings = [new_pairings; tracklet_id_1 tracklet_id_2];  
    end
  end

  correspondences.tracklet_pairings = new_pairings;
  size(new_pairings,1);

end
