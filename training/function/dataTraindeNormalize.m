function data_n = dataTraindeNormalize(data,min,range)

data_n = (data .* range) + min;

end

