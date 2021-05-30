clear all
close all
clc

fid= fopen('../Initial_Material/controllers/localization_supervisor/errors.csv');
data= textscan(fid,"%f %f %f %f %f %f %f %f %f","Delimiter",";","headerlines",1);

time= data{:,1};
gps_error= data{:,2};
acc_error= data{:,3};
enc_error= data{:,4};
kal_error= data{:,5};
gps_mean= data{:,6};
acc_mean= data{:,7};
% enc_mean= data{:,8};
% kal_mean= data{:,9};

figure("name","ERRORS")
hold on
% plot(time,gps_error,"displayname","gps error");
% plot(time,acc_error,"displayname","acc error");
% plot(time,enc_error,"displayname","enc error");
% plot(time,kal_error,"displayname","kal error");

plot(gps_error,"displayname","gps error");
plot(acc_error,"displayname","acc error");
plot(enc_error,"displayname","enc error");
plot(kal_error,"displayname","kal error");

legend show

fclose(fid);