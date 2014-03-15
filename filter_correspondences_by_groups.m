function [shapes] =  filter_correspondences_by_groups(correspondences, ...
                                         time_scaling_factor, ...
                                         bins)

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

  max_cam_num = 0;
  active_cameras = [];
  for i=1:size(correspondences.tracklets_cam_coords)
    cam_num = correspondences.tracklets_cam_coords{i}.cam_num;
    if(cam_num > max_cam_num)
      max_cam_num = cam_num;
    end
    if(!ismember(cam_num, active_cameras))
      active_cameras = [active_cameras cam_num];
    end
  end

  num_tracklets = size(correspondences.tracklets_cam_coords);

  % create array of every point for each camera
  %camera_points = cell(correspondences.num_cameras);
  camera_points = cell(max_cam_num);
  for i=1:max_cam_num
    camera_points{i} = [];
  end

  % populate camera_points array
  for i=1:num_tracklets
    tracklet = correspondences.tracklets_cam_coords{i};
    % append the tracklet index to each point, will be needed to get the tracklet
    % number from a kdtree search result
    points = [tracklet.path ones(size(tracklet.path,1),1)*i];
    camera_points{tracklet.cam_num} = [camera_points{tracklet.cam_num}; points];
  end

  % build a KD-tree for each camera
  kdtrees = cell(max_cam_num);
  for i=1:size(active_cameras,1)
    cam = active_cameras(i);
    % ignore the tracklet index field in the KD tree
    kdtrees{cam} = KDTreeSearcher(camera_points{cam}(:,1:3), 'euclidean');
  end

  % create and initialize the shape histogram for each tracklet
  shapes = cell(num_tracklets);

  % create shape for each tracklet
  for trackletId=1:num_tracklets
    tracklet = correspondences.tracklets_cam_coords{i};
    cam_num = tracklet.cam_num;
      
    accumulatedHistogram = zeros(size(bins,1),1);

    % create shape for each point on tracklet and take average of all points'
    % shapes to get the shape for the tracklet
    for i=1:size(tracklet.path,1)
      curPoint = tracklet.path(i,:);
      % TODO this can be made more efficient with only one kdtree range querey
      curHistogram = zeros(size(bins,1),1);
      matches = [];

      % populate histogram for each bin
      for r=1:size(bins,1)
        radius = bins(r);
        cur_bin_count = 0;
        indexes = rangesearch(kdtrees{cam_num}, curPoint, radius);

        % for each result contained in indexes, push it into the
        % matches array to keep track of which tracklets we 
        % have found a result from
        for l=1:size(indexes{1},1)
          found = 0;
          % check if the tracklet the result belongs to has already
          % been counted in a previous result
          for k=1:size(matches,1)
            if(camera_points{cam_num}(matches(k),4) == cam_num)
              found = 1;
              break;
            end
          end % for k=1:size(matches,1)

          % if from a new tracklet, push to matches and increase
          % bin count for this radius
          if(found == 0)
            matches = [matches; l];
            cur_bin_count = cur_bin_count + 1;
          end

        end % for l=1:size(indexes{1},1)

        curHistogram(r) = cur_bin_count;
      end % for r=1:size(bins,1)

      accumulatedHistogram = accumulatedHistogram + curHistogram;
    end
    
    % the shape of the whole tracklet is the average of the shape of each
    % point in the tracklet
    accumulatedHistogram = accumulatedHistogram/size(tracklet.path,1);
    shapes{trackletId} = accumulatedHistogram;
  end

end
