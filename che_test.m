clear
clc
close all
figure(1)

% ONLY WORKS FOR CAMERAS 1,18,19 AT THE MOMENT...

hold on
win_len = 100;
[all_corrs,gt,dt] = load_external_data('./raw_data/filtered_data/a.out',win_len);
track_objs = all_corrs.tracklets_cam_coords;
corrs = all_corrs.tracklet_pairings;
disp(sprintf('Num Cameras: %d',all_corrs.num_cameras));
for i = 1:length(track_objs)
        
    if (max(max(i == corrs(:,1))) == 1)
        switch (track_objs{i}.cam_num)
            case 1,
                plot(track_objs{i}.path(:,3), track_objs{i}.path(:,1),'-c')
            case 18,
                plot(track_objs{i}.path(:,3), track_objs{i}.path(:,1),'-y')
            case 19,
                 plot(track_objs{i}.path(:,3), track_objs{i}.path(:,1),'-m')
        end
    elseif (max(max(i == corrs(:,2))) == 1)
        switch (track_objs{i}.cam_num)
            case 1,
                plot(track_objs{i}.path(:,3), track_objs{i}.path(:,1),'-k')
            case 18,
                plot(track_objs{i}.path(:,3), track_objs{i}.path(:,1),'-k')
            case 19,
                plot(track_objs{i}.path(:,3), track_objs{i}.path(:,1),'-k')
        end
    else
        switch (track_objs{i}.cam_num)
            case 1,
                plot(track_objs{i}.path(:,3), track_objs{i}.path(:,1),'-b')
            case 18,
                plot(track_objs{i}.path(:,3), track_objs{i}.path(:,1),'-g')
            case 19,
                plot(track_objs{i}.path(:,3), track_objs{i}.path(:,1),'-r')
        end
    end
    
%     disp(sprintf('Init Cam Num: %d',all_corrs.tracklets_cam_coords{i}.cam_num))
    if (all_corrs.tracklets_cam_coords{i}.cam_num == 1)
        all_corrs.tracklets_cam_coords{i}.cam_num = 1;
    elseif (all_corrs.tracklets_cam_coords{i}.cam_num == 18)
        all_corrs.tracklets_cam_coords{i}.cam_num = 2;
    elseif (all_corrs.tracklets_cam_coords{i}.cam_num == 19)
        all_corrs.tracklets_cam_coords{i}.cam_num = 3;
    end
%     disp(sprintf('Final Cam Num: %d\n',all_corrs.tracklets_cam_coords{i}.cam_num))
    
end

disp(sprintf('\ndt = %d, win_t = %f\n',dt,win_len*dt))
for i = 1:size(corrs,1)
    track1 = track_objs{corrs(i,1)};
    track2 = track_objs{corrs(i,2)};
    
    
    % Case 0 - Complete Overlap Tr1 in Tr2
    if ((track1.first_time >= track2.first_time) & (track1.last_time <= track2.last_time))
        disp(sprintf('Case 0: Tr1 in Tr2'));
    % Case 1 - Complete Overlap Tr2 in Tr1
    elseif ((track1.first_time <= track2.first_time) & (track1.last_time >= track2.last_time))
        disp(sprintf('Case 1: Tr2 in Tr1'));
    % Case 2 - Partial Overlap Tr1 <= Tr2
    elseif ((track1.last_time >= track2.first_time) & (track1.last_time < track2.last_time))
        disp(sprintf('Case 2: Partial Overlap (Tr1 <= Tr2)'))
    % Case 3 - Partial Overlap Tr1 >= Tr2
    elseif ((track1.first_time <= track2.last_time) & (track1.last_time > track2.last_time))
        disp(sprintf('Case 3: Partial Overlap (Tr2 >= Tr2)'))
    % Case 4 - Tr1 < Tr2, but within win_len*dt
    elseif (track1.last_time < track2.first_time)
        disp(sprintf('Case 4:\nDist = %f', track2.first_time - track1.last_time));
    % Case 5 - Tr1 > Tr2, but within win_len*dt
    elseif (track1.first_time > track2.last_time)
        disp(sprintf('Case 5:\nDist = %f', track1.first_time - track2.last_time));    
    end
    
    disp(sprintf('T1: %d,%d\nT2: %d,%d\n',track1.first_time,track1.last_time, ...
                                          track2.first_time,track2.last_time))
end

correspondences = all_corrs;
ground_truth = gt;

save test_data.mat correspondences ground_truth

hold off