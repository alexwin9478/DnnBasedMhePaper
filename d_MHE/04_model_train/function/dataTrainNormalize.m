function [data_n, min_n, range] = dataTrainNormalize(data)
min_n = min(data);
range = max(data) - min(data);
data_n = (data - min(data))./ range;
end
