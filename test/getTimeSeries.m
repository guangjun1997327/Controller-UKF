function X = getTimeSeries(signal, fieldnames, data)
%% Skript to get timeseries out of data vector from testing data
% Input:    signal      name of the signal you want to convert
%           fieldnames  names of all the signal available
%           data        data vector with all the signals
% get testing time vector
time = data.t;

% get data for the wanted signal
for k = 1:length(fieldnames)
    % find fieldname that fits the wanted signal
    if strcmp(fieldnames{k}, signal)
        % write data into timeseries
        time_data = eval(strcat('data.', signal));
    end
end
% generate timeseries from data and time
X = timeseries(single(time_data),time);
end