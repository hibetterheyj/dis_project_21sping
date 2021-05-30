clc; clear all; close all;

temp = importdata('../Initial_Material/controllers/localization_controller/logs.csv');

data = [];
ind = 0;
for f_ = str2mat(temp.textdata)'
    ind = ind + 1;
    data.(strrep(f_',' ','')) = temp.data(:,ind);
end

% store acceleration into an array
data.acc = [data.acc_0,data.acc_1,data.acc_2];

N_SIM = length(data.time);
T_SIM = 1: N_SIM;
T = 0.016;