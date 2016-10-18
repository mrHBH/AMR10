%% HS AMR TU Dresden
% Script for visualization and analysis of NXT logging data
% Author: IfA
clear all; clc;

%% User inputs
% ADJUST: file name
file = fopen('NXTData.txt', 'r');
% ADJUST: identifier (in order to specify line type and module)
% id = [<module number> <logging data type>]
% module number: 0 (control), 1 (guidance), 2 (hmi), 3 (navigation), 4 (perception)
% logging data type: 0 (chart), 1 (comment strings)
id = [0 0];

%% Execution part - do not change without specific goals
output = {}; % output cell array
rows = 0; % number of rows of output matrix (number of log entries)
while 1 % read file line by line
    logEntry = fgetl(file); % get new line (log entry)
    if logEntry == -1 % stop if end of file reached
       break; 
    end
    if (isempty(strfind(logEntry,';'))) | ~(isempty(strfind(logEntry,';;'))) % ignore obsolete log entry and time stamp of NXTDataLogger, ignore empty logs
        continue;
    end
    logEntrySubStrings = regexp(logEntry,';','split'); % split log entry into substrings
    if ~(str2double(logEntrySubStrings(2)) == id(1) & (str2double(logEntrySubStrings(3)) == id(2))) % ignore log entry that do not conform to id
        continue;
    end
    if id(2) == 1
        disp(logEntry) % visualization of comment strings (console)
        continue;
    end
    rows = rows+1;
    output{rows,1} = char(logEntrySubStrings(1)); % first column = timestamp
    for k = 4:(size(logEntrySubStrings,2)) % read substring by substring; skip id
        output{rows,k-2} = char(logEntrySubStrings(k)); % add element to output matrix
    end
end
if id(2) == 0 % visualization of chart (plots)
    output = cellfun(@str2double, output); % convert cell array to numeric matrix
    figure(1);
    n = size(output,2);
    for m = 2:n
        subplot(n-1,1,m-1); plot(output(:,1),output(:,m)); zoom on; grid on; ylabel({'chart column';m});
    end
    xlabel({'Zeit in ms';'chart column';1});
end