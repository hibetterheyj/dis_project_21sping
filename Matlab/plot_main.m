%% Part A : Plot accelerometers values 

[N_SIM, T_SIM, T, data] = read_log();

f = figure('Name','accelerometer [m/s^2]');

% Plot x acceleration 
subplot(3,1,1);
plot(data.time , data.acc(:,1));
title('acc[0]');
xlabel('Time [s]'); ylabel('acc [m/s^2]');
y_lim = [min( data.acc(:,1)),max(data.acc(:,1))];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

% Plot y acceleration
subplot(3,1,2);
plot(data.time , data.acc(:,2));
title('acc[1]');
xlabel('Time [s]'); ylabel('acc [m/s^2]');
y_lim = [min(data.acc(:,2)),max(data.acc(:,2))];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

% Plot z acceleration
subplot(3,1,3);
plot(data.time , data.acc(:,3));
title('acc[2]');
xlabel('Time [s]'); ylabel('acc [m/s^2]');
y_lim = [min(data.acc(:,3)),max(data.acc(:,3))];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part B : Compute and plot the odometry computed using the accelerometer

[N_SIM, T_SIM, T, data] = read_log();

odo.x       = zeros(N_SIM + 1,1);
odo.vx      = zeros(N_SIM + 1,1);

% Compute the static biais (mean)
data.acc_mean = mean(data.acc(1/T:5/T,:));

% Compute the odometry
for t_ = T_SIM
    [odo.x(t_ + 1), odo.vx(t_ + 1)] = ...
    odo_acc(odo.x(t_), odo.vx(t_), T, data.acc(t_,:), data.acc_mean);
end

% Plot the odometry computed using the accelerometer
f = figure('Name','Odometry using accelerometer [m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(data.time, data.pose_x); hold on;
plot(data.time, odo.x(2:end));
title('x trajectory : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Accelerometer');
xlabel('Time [s]'); ylabel('x [m]');
y_lim = [min([odo.x(2:end);  data.pose_x]),max([odo.x(2:end);  data.pose_x])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part C : Plot the odometry computed using the accelerometer on Webots

[N_SIM, T_SIM, T, data] = read_log();

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(data.time, data.pose_x); hold on;
plot(data.time, data.odo_acc_x);
title('x trajectory : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Accelerometer');
xlabel('Time [s]'); ylabel('x [m]');
y_lim = [min([data.odo_acc_x;  data.pose_x]),max([data.odo_acc_x;  data.pose_x])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part D : Compute and plot the odometry computed using the wheel encoders

[N_SIM, T_SIM, T, data] = read_log();

odo.x       = zeros(N_SIM ,1);
odo.y       = zeros(N_SIM ,1);
odo.heading = zeros(N_SIM ,1);

% Compute Delta wheel encoders
data.A_enc_r = diff(data.right_enc);
data.A_enc_l = diff(data.left_enc); 

% Compute the odometry
for t_ = T_SIM(1:end-1)
    [odo.x(t_ + 1), odo.y(t_ + 1), odo.heading(t_ + 1)] = ...
     odo_enc(odo.x(t_), odo.y(t_), odo.heading(t_), T, data.A_enc_r(t_), data.A_enc_l(t_));
end

% Plot the odometry computed using the accelerometer
f = figure('Name','Odometry using wheel encoders [Rad]'); 
subplot(3,1,[1 2]);hold on;

% Plot x -y plan : odometry vs ground truth (gps)
plot(data.pose_x(2:end) , data.pose_y(2:end)); hold on;
plot(odo.x(2:end) , odo.y(2:end));
title('x -y plan : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Wheel encoders');
xlabel('x [m]'); ylabel('y [m]');
x_lim = [min([odo.x(2:end);  data.pose_x]),max([odo.x(2:end);  data.pose_x])];
y_lim = [min([odo.y(2:end);  data.pose_y]),max([odo.y(2:end);  data.pose_y])];
xlim(x_lim + [-0.05,0.05]*(x_lim(2)-x_lim(1)));ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
axis equal;

% Plot heading : odometry vs ground truth (gps)
subplot(3,1,3);hold on;
plot(data.time ,  data.pose_heading); hold on;
plot(data.time , wrapToPi(odo.heading));
title('Heading : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Wheel encoders');
xlabel('Time [s]'); ylabel('heading [Rad]');
y_lim = [min([wrapToPi(odo.heading(2:end));  data.pose_heading]),max([wrapToPi(odo.heading(2:end));  data.pose_heading])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part E : Plot the odometry computed using the wheel encoders on Webots

[N_SIM, T_SIM, T, data] = read_log();

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using wheel encoders [Rad]'); 
subplot(3,1,[1 2]);hold on;

% Plot x -y plan : odometry vs ground truth (gps)
plot(data.pose_x(2:end) , data.pose_y(2:end)); hold on;
plot(data.odo_enc_x , data.odo_enc_y);
title('x -y plan : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Wheel encoders');
xlabel('x [m]'); ylabel('y [m]');
x_lim = [min([data.odo_enc_x;  data.pose_x]),max([data.odo_enc_x;  data.pose_x])];
y_lim = [min([data.odo_enc_y;  data.pose_y]),max([data.odo_enc_y;  data.pose_y])];
xlim(x_lim + [-0.05,0.05]*(x_lim(2)-x_lim(1)));ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
axis equal;

% Plot heading : odometry vs ground truth (gps)
subplot(3,1,3);hold on;
plot(data.time ,  data.pose_heading); hold on;
plot(data.time , wrapToPi(data.odo_enc_heading));
title('Heading : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Wheel encoders');
xlabel('Time [s]'); ylabel('heading [Rad]');
y_lim = [min([wrapToPi(data.odo_enc_heading);  data.pose_heading]),max([wrapToPi(data.odo_enc_heading);  data.pose_heading])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));