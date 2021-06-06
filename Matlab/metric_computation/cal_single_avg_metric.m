clear all
close all
clc

% For flocking metric
fid= fopen('obstacle_data/manually/flocking_metrics0.csv');
data= textscan(fid,"%f %f %f %f %f","Delimiter",";","headerlines",1); 
%get values
segment = 2:939;
time= data{:,1};
metrics = data{:,5};
 
% % For formation metric
% data= textscan(fid,"%f %f %f %f","Delimiter",";","headerlines",1); 
% %get values
% segment = 2:939;
% time= data{:,1};
% metrics = data{:,4};



time = time(segment);
metrics = metrics(segment);

metrics_avg = mean(metrics);