%% user set params

%bag_name = '~/Documents/bags/bebop_sitl_ident_slow1_2019-05-28-15-41-47.bag'; 
%bag_name = '/home/lmark/Documents/bags/real_ident/bag_estimacija_lovro.bag';
bag_name = '/home/lmark/Documents/bags/arducopter/arducopter_yaw.bag';
%imu_topic = '/mavros/imu/data';
imu_topic = '/uav/imu';
%control_topic = '/sim/attitude_sp';
control_topic = '/uav/rpy_thrust';

%bag_name = '/home/lmark/Documents/bags/real_ident/neo_bez_rukica.bag';
%imu_topic = '/euroc3/imu';
%control_topic = '/euroc3/command/roll_pitch_yawrate_thrust';

%% read bag file
path(path, '../read_bags');
path(path, '../helper_functions');

%close all;
clc;

bag = ros.Bag(bag_name);
bag.info

imu_data = readImu(bag, imu_topic);
attitude_cmd = readCommandRollPitchYawRateThrust(bag, control_topic);
%%

imu_data.rpy = quat2rpy([imu_data.q(4,:)', imu_data.q(1:3,:)']');
attitude_cmd.rpy = vertcat(attitude_cmd.roll, attitude_cmd.pitch, attitude_cmd.yaw_rate);

t_start = imu_data.t(1);
imu_data.t = imu_data.t - t_start;
attitude_cmd.t = attitude_cmd.t - attitude_cmd.t(1);
%% plot
figure(1);
ax = axes;
plot(imu_data.t, imu_data.rpy(1,:), 'linewidth', 2);
hold on;
plot(attitude_cmd.t, attitude_cmd.rpy(1,:), '--', 'linewidth', 2);
xlabel('time');
ylabel('roll [rad]');
title('roll angle');
legend('\phi imu', '\phi cmd');
grid on;
ax.FontSize = 16;

figure(2);
ax = axes;
plot(imu_data.t, imu_data.rpy(2,:), 'linewidth', 2);
hold on;
plot(attitude_cmd.t, attitude_cmd.rpy(2,:), '--', 'linewidth', 2);
xlabel('time');
ylabel('\theta [rad]');
title('pitch angle');
legend('\theta imu', '\theta cmd');
grid on;
ax.FontSize = 16;

figure(3);
ax = axes;
plot(imu_data.t, imu_data.a(3, :), 'linewidth', 2);
hold on;
plot(attitude_cmd.t, attitude_cmd.thrust, '--', 'linewidth', 2);
xlabel('time');
ylabel('???');
title('Thrust');
legend('Acceleration', 'Commanded thrust');
grid on;
ax.FontSize = 16;

figure(4);
ax = axes;
plot(imu_data.t, imu_data.rpy(3,:), 'linewidth', 2);
hold on;
plot(attitude_cmd.t, attitude_cmd.rpy(3,:), '--', 'linewidth', 2);
xlabel('time');
ylabel('\theta [rad]');
title('yaw angle');
legend('\theta imu', '\theta cmd');
grid on;
ax.FontSize = 16;
%% sysid

%sys_id_start_time_s = 25;
%sys_id_end_time_s = 165;

sys_id_start_time_s = 0;
sys_id_end_time_s = 42;

attitude_cmd.rpy_interp = zeros(size(imu_data.rpy));
attitude_cmd.rpy_interp(1,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(1,:), imu_data.t);
attitude_cmd.rpy_interp(2,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(2,:), imu_data.t);
attitude_cmd.rpy_interp(3,:) = interp1(attitude_cmd.t, attitude_cmd.rpy(3,:), imu_data.t);

attitude_cmd.t = imu_data.t;

imu_data.t = imu_data.t(imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);
imu_data.rpy = imu_data.rpy(:, imu_data.t > sys_id_start_time_s & imu_data.t < sys_id_end_time_s);

attitude_cmd.t = attitude_cmd.t(attitude_cmd.t > sys_id_start_time_s & attitude_cmd.t < sys_id_end_time_s);
attitude_cmd.rpy_interp = attitude_cmd.rpy_interp(:, attitude_cmd.t > sys_id_start_time_s & attitude_cmd.t < sys_id_end_time_s);

dt = mean(diff(imu_data.t));
roll_data = iddata(imu_data.rpy(1,:)', attitude_cmd.rpy_interp(1,:)', dt);
pitch_data = iddata(imu_data.rpy(2,:)', attitude_cmd.rpy_interp(2,:)', dt);
yaw_data = iddata(imu_data.rpy(3,:)', attitude_cmd.rpy_interp(3,:)', dt);

%%%%%% Dodano
roll_data =  detrend(roll_data);
pitch_data = detrend(pitch_data);
yaw_data = detrend(yaw_data);
%%%%%%%

roll_tf_1 = tfest( roll_data, 1, 0);
pitch_tf_1 = tfest( pitch_data, 1, 0);
yaw_tf_1 = tfest(yaw_data, 1, 0);

disp('======================');
disp('roll 1st order dynamics'); 
% roll_tf
fprintf('roll_gain: %f\n\n', dcgain(roll_tf_1));
fprintf('roll_time_constant: %f\n\n', -1/pole(roll_tf_1));
fprintf('fit percentage: %f %%\n', roll_tf_1.Report.Fit.FitPercent);
disp('----------------------');
disp('pitch 1st order dynamics'); 
% pitch_tf
fprintf('pitch_gain: %f\n\n', dcgain(pitch_tf_1));
fprintf('pitch_time_constant: %f\n\n', -1/pole(pitch_tf_1));
fprintf('fit percentage: %f\n', pitch_tf_1.Report.Fit.FitPercent);
% yaw_tf
disp('----------------------');
disp('yaw 1st order dynamics'); 
fprintf('yaw_gain: %f\n\n', dcgain(yaw_tf_1));
fprintf('yaw_time_constant: %f\n\n', -1/pole(yaw_tf_1));
fprintf('fit percentage: %f\n', yaw_tf_1.Report.Fit.FitPercent);


%2nd order
roll_tf_2 = tfest( roll_data, 2, 0);
pitch_tf_2 = tfest( pitch_data, 2, 0);
yaw_tf_2 = tfest(yaw_data, 2, 0);

disp('----------------------');
disp('roll 2st order dynamics:'); 
% roll_tf
[Wn, damping] = damp(roll_tf_2);
fprintf('roll_gain: %f\n\n', dcgain(roll_tf_2));
fprintf('roll_damping_coef: %f\n\n', damping(1));
fprintf('roll_natural_freq: %f\n\n', Wn(1));
fprintf('fit percentage: %f\n', roll_tf_2.Report.Fit.FitPercent);

disp('----------------------');
disp('pitch 2st order dynamics:'); 
% pitch_tf
[Wn, damping] = damp(pitch_tf_2);
fprintf('pitch_gain: %f\n\n', dcgain(pitch_tf_2));
fprintf('pitch_damping_coef: %f\n\n', damping(1));
fprintf('pitch_natural_freq: %f\n\n', Wn(1));
fprintf('fit percentage: %f\n', pitch_tf_2.Report.Fit.FitPercent);

disp('----------------------');
disp('yaw 2st order dynamics:'); 
% pitch_tf
[Wn, damping] = damp(yaw_tf_2);
fprintf('yaw_gain: %f\n\n', dcgain(yaw_tf_2));
fprintf('yaw_damping_coef: %f\n\n', damping(1));
fprintf('yaw_natural_freq: %f\n\n', Wn(1));
fprintf('fit percentage: %f\n', yaw_tf_2.Report.Fit.FitPercent);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Do the validation

valid_file =  '~/Documents/bags/bebop_sitl_ident2_2019-05-28-15-22-36.bag';

valid_bag = ros.Bag(valid_file);
valid_bag.info

imu_valid = readImu(valid_bag, imu_topic);
attitude_cmd_valid = readCommandRollPitchYawRateThrust(valid_bag, control_topic);

%%

imu_valid.rpy = quat2rpy([imu_valid.q(4,:)', imu_valid.q(1:3,:)']');
attitude_cmd_valid.rpy = vertcat(attitude_cmd_valid.roll, attitude_cmd_valid.pitch, attitude_cmd_valid.yaw_rate);

t_start = imu_valid.t(1);
imu_valid.t = imu_valid.t - t_start;
attitude_cmd_valid.t = attitude_cmd_valid.t - attitude_cmd_valid.t(1);

%% plot valid

figure(1);
ax = axes;
plot(imu_valid.t, imu_valid.rpy(1,:), 'linewidth', 2);
hold on;
plot(attitude_cmd_valid.t, attitude_cmd_valid.rpy(1,:), '--', 'linewidth', 2);
xlabel('time');
ylabel('roll [rad]');
title('roll angle');
legend('\phi imu', '\phi cmd');
grid on;
ax.FontSize = 16;

figure(2);
ax = axes;
plot(imu_valid.t, imu_valid.rpy(2,:), 'linewidth', 2);
hold on;
plot(attitude_cmd_valid.t, attitude_cmd_valid.rpy(2,:), '--', 'linewidth', 2);
xlabel('time');
ylabel('\theta [rad]');
title('pitch angle');
legend('\theta imu', '\theta cmd');
grid on;
ax.FontSize = 16;

figure(3);
ax = axes;
plot(imu_valid.t, imu_valid.a(3, :), 'linewidth', 2);
hold on;
plot(attitude_cmd_valid.t, attitude_cmd_valid.thrust, '--', 'linewidth', 2);
xlabel('time');
ylabel('???');
title('Thrust');
legend('Acceleration', 'Commanded thrust');
grid on;
ax.FontSize = 16;

%% valid sysid

valid_start_time_s = 5;
valid_end_time_s = 50;

attitude_cmd_valid.rpy_interp = zeros(size(imu_valid.rpy));
attitude_cmd_valid.rpy_interp(1,:) = interp1(attitude_cmd_valid.t, attitude_cmd_valid.rpy(1,:), imu_valid.t);
attitude_cmd_valid.rpy_interp(2,:) = interp1(attitude_cmd_valid.t, attitude_cmd_valid.rpy(2,:), imu_valid.t);
attitude_cmd_valid.rpy_interp(3,:) = interp1(attitude_cmd_valid.t, attitude_cmd_valid.rpy(3,:), imu_valid.t);

attitude_cmd_valid.t = imu_valid.t;

imu_valid.t = imu_valid.t(imu_valid.t > valid_start_time_s & imu_valid.t < valid_end_time_s);
imu_valid.rpy = imu_valid.rpy(:, imu_valid.t > valid_start_time_s & imu_valid.t < valid_end_time_s);

attitude_cmd_valid.t = attitude_cmd_valid.t(attitude_cmd_valid.t > valid_start_time_s & attitude_cmd_valid.t < valid_end_time_s);
attitude_cmd_valid.rpy_interp = attitude_cmd_valid.rpy_interp(:, attitude_cmd_valid.t > valid_start_time_s & attitude_cmd_valid.t < valid_end_time_s);

dt_valid = mean(diff(imu_valid.t));
roll_valid = iddata(imu_valid.rpy(1,:)', attitude_cmd_valid.rpy_interp(1,:)', dt_valid);
pitch_valid = iddata(imu_valid.rpy(2,:)', attitude_cmd_valid.rpy_interp(2,:)', dt_valid);

%%%%%% Dodano
roll_valid =  detrend(roll_valid);
pitch_valid = detrend(pitch_valid);
%%%%%%%

%% Validate dataset - roll

figure;
compare(roll_valid, roll_tf_1);
[~, fit1_roll, ~] = compare(roll_valid, roll_tf_1);
disp(strcat('The roll model (1st order) fits the validation data with **', ...
       num2str(fit1_roll), '** %'));

figure;
compare(roll_valid, roll_tf_2);
[~, fit2_roll, ~] = compare(roll_valid, roll_tf_2);
disp(strcat('The roll model (2nd order) fits the validation data with **', ...
       num2str(fit2_roll), '** %'));

%% Validate dataset - pitch

figure;
compare(pitch_valid, pitch_tf_1);
[~, fit1_pitch, ~] = compare(pitch_valid, pitch_tf_1);
disp(strcat('The roll model (1st order) fits the validation data with **', ...
       num2str(fit1_pitch), '** %'));

figure;
compare(pitch_valid, pitch_tf_2);
[~, fit2_pitch, ~] = compare(pitch_valid, pitch_tf_2);
disp(strcat('The roll model (2nd order) fits the validation data with **', ...
       num2str(fit2_pitch), '** %'));