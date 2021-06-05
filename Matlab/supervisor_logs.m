clear all
close all
clc
%% results plotting
set(groot,'DefaultLineLineWidth', 1.5)
%opening and loading logs file
fid= fopen('../Initial_Material/controllers/localization_supervisor/errors.csv');
data= textscan(fid,"%f %f %f %f %f %f %f %f %f %f %f","Delimiter",";","headerlines",1);

time= data{:,1};
gps_error= data{:,2};
acc_error= data{:,3};
enc_error= data{:,4};
kal1_error= data{:,5};
kal2_error= data{:,6};
gps_mean= data{:,7};
acc_mean= data{:,8};
enc_mean= data{:,9};
kal1_mean= data{:,10};
kal2_mean= data{:,11};

%plotting metrics
figure("name","ERRORS")
title("Localization Error");
hold on
plot(time,gps_error,"displayname","gps error");
plot(time,acc_error,"displayname","acc error");
plot(time,enc_error,"displayname","enc error");
plot(time,kal1_error,"displayname","kal acc error");
plot(time,kal2_error,"displayname","kal enc error");
xlabel("time (s)")
ylabel("error metric value")
legend show
legend("location","northwest")

fclose(fid);