function [ min_t, max_t, len_t ] = trackblob_get_timeinfo( trackblob )
%TRACKBLOB_GET_TIMEINFO Find the max length of time that stuff is
% visible

min_t = inf;
max_t = -inf;

for i = 1:length(trackblob.tracklets)
    min_t = min(min_t, trackblob.tracklets{i}.first_time);
    max_t = max(max_t, trackblob.tracklets{i}.first_time + size(trackblob.tracklets{i}.path,1) - 1);
end

len_t = max_t - min_t;

end

