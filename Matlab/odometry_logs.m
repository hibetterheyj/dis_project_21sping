clear all
close all
clc
addpath("functions")
% If accelerometer positioning is wrong, make clean and rebuild the 
% controller (often occurs after the first build of a new modification in 
% the code)
%% results plotting
%opening and loading logs file
fid= fopen('../Initial_Material/controllers/localization_controller/logs.csv');
data= textscan(fid,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f","Delimiter",";","headerlines",1);

pose_x= data{:,2};
pose_y= data{:,3};
pose_head= data{:,4};

gps_x= data{:,5};
gps_y= data{:,7};

odo_acc_x= data{:,13}; 
odo_acc_y= data{:,14};
odo_acc_head= data{:,15};

odo_enc_x= data{:,16}; 
odo_enc_y= data{:,17};

kal_acc_x= data{:,19};
kal_acc_y= data{:,20}; 

kal_enc_x= data{:,21};
kal_enc_y= data{:,22}; 

%plotting positions
figure;
plot(pose_x,pose_y,"displayname","Pose");
hold on
plot(gps_x,-gps_y,"displayname","GPS");
plot(odo_acc_x,odo_acc_y,"displayname","ACC");
plot(odo_enc_x,odo_enc_y,"displayname","ENC");
plot(kal_acc_x,kal_acc_y,"displayname","Kalman ACC");
plot(kal_enc_x,kal_enc_y,"displayname","Kalman ENC");
title("Epuck localization, various methods");
xlabel("x axis")
ylabel("y axis")

legend show
%% kalman strategies testing
acc_0= data{:,8};
acc_1= data{:,9};
acc_2= data{:,10};

enc_right= data{:,11};
enc_left= data{:,12};
time= data{:,1};
time= time;

%test of kalman functions before implementing in c
%[X, Cov]= kalman_function(acc_1,acc_0,pose_x,pose_y,time, pose_x, pose_y, pose_head, odo_acc_head)
%[X, Cov]= kalman_function2(enc_left,enc_right,pose_x,pose_y,time, pose_x, pose_y, pose_head, pose_head)