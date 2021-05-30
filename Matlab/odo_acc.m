function [x, vx] = odo_acc(x, vx, T, acc, acc_mean)
% Express the acceleration in Webots world frame and remove the bias (Assume 1-D) 
% To Do : Complete the following equation. Make sure to use the correct index of the acc.

acc_wx =(acc(2) - acc_mean(2));

% Odometry in X 
% To Do : Complete the following equations using the previous acc_wx
vx = vx + acc_wx * T;

x = x + vx * T;

end