clear all;
clc;

addpath('Matlab plots\');
addpath('Matlab plots\DiffTune_plus');

%% Frequency for reference sine wave
% Frequency for reference sine wave
xf = 1;

%% Parameters for drive train
% This is the initialization script for the motor and axle parameters. Both
% motors are identical PMSM 1FT7042-5AF70-1DA0 HD.

% Motor and load mechanical parameters
N = 1;              % -- Gear ratio
J_m = 8.31e-4;      % kg m^2 -- Moment of inertia
J_l = 8.31e-4;      % kg m^2 -- Moment of inertia

% Common simulation parameters
T_s = 0.000125;
T_log = 0.001;

% Taken from Table 4.3: Summary of calculated friction and shaft parameters
% (page 40, Dimitrios Papageorgiou phd thesis)
K_S = 32.94;        % N m rad^(-1)
D_S = 0.0548;       % N m s rad^(-1)
T_Cm = 0.0223;      % N m
T_Cl = 0.0232;      % N m
beta_m = 0.0016;    % N m s rad^(-1)
beta_l = 0.0016;    % N m s rad^(-1)

% Initial conditions vector (should be zero)
x_0 = [0,0];
x_l_0 = [0,0];

%% Controller gains
% From DiffTune+
k1 = 2.433;
k2 = 0.965;
k_pos = 11.86;

%%
d_e_mag = 0;

% Simulink simulation - STSMC and P-STSMC hand-tuning
driveTrain_sim = sim('driveTrain_P_STSMC', 10);

% Extracting data
omega_r_timeseries = driveTrain_sim.omega_r_out;
theta_r_timeseries = driveTrain_sim.theta_r_out;
omega_m_timeseries = driveTrain_sim.omega_m_out;
theta_l_timeseries = driveTrain_sim.theta_l_out;

% Extracting data not as timeseries
time = omega_r_timeseries.Time;
omega_r = omega_r_timeseries.Data;
theta_r = theta_r_timeseries.Data;
omega_m = omega_m_timeseries.Data;
theta_l = theta_l_timeseries.Data;

% Loss and RSME calculations (same as used for DiffTune)
e_theta = theta_r - theta_l;
loss_theta = e_theta .^ 2;
acc_loss_theta = sum(loss_theta);   % accumulated loss
rmse_theta = sqrt(1/length(time) * acc_loss_theta);

h2 = figure(2);

subplot(2,2,[1 2]);
plot(driveTrain_sim.theta_l_out, 'LineWidth', 1.5);
hold on;
plot(driveTrain_sim.theta_r_out, '--', 'LineWidth', 1.5);
hold off;
grid on;
legend('\theta_l', '\theta_r', 'Location', 'southeast');
ylim([-1 1]);
xlabel('time (s)');
ylabel('position (rad)');
text(0.5,-0.1,['k1 = ' sprintf('%.3f', k1)]);
text(0.5,-0.3,['k2 = ' sprintf('%.3f', k2)]);
text(0.5,-0.5,['k_pos = ' sprintf('%.2f', k_pos)]);
text(0.5,-0.7,['rmse = ' sprintf('%.4f', rmse_theta)]);
title('DiffTune+ auto-tuned P-STSMC sine response');

subplot(2,2,3);
plot(time, abs(e_theta)*10^3, 'LineWidth', 1.5);
grid on;
legend('|e_\theta|', 'Location', 'northeast');
xlabel('time (s)');
ylabel('position error (mrad)');
title('Position error');

subplot(2,2,4);
plot(driveTrain_sim.T_m_out, 'LineWidth', 1.5);
hold on;
yline(2.7,'--');
hold off;
grid on;
legend('u', 'rated torque', 'Location', 'northeast');
xlabel('time (s)');
ylabel('torque (N m)');
title('Motor torque');

saveas(h2, 'Matlab plots\DiffTune_plus\P-STSMC DiffTune_plus check.png');

%% Simulink simulation - STSMC and P-STSMC hand-tuning
h1 = figure(1);

% Amplitude of disturbance sine wave
% d_e_mags = [0 0.02:0.8:6];
d_e_mags = 0.02:0.2:1;

% Initialize a cell array to store legend entries
legend_entries = cell(1, length(d_e_mags));

for i = 1 : length(d_e_mags)
    % Simulate
    d_e_mag = d_e_mags(i);
    driveTrain_sim = sim('driveTrain_P_STSMC', 10);

    % Extracting data
    omega_r_timeseries = driveTrain_sim.omega_r_out;
    theta_r_timeseries = driveTrain_sim.theta_r_out;
    omega_m_timeseries = driveTrain_sim.omega_m_out;
    theta_l_timeseries = driveTrain_sim.theta_l_out;

    % Extracting data not as timeseries
    time = omega_r_timeseries.Time;
    omega_r = omega_r_timeseries.Data;
    theta_r = theta_r_timeseries.Data;
    omega_m = omega_m_timeseries.Data;
    theta_l = theta_l_timeseries.Data;

    % Loss and RSME calculations (same as used for DiffTune)
    e_theta = theta_r - theta_l;
    loss_theta = e_theta .^ 2;
    acc_loss_theta = sum(loss_theta);   % accumulated loss
    rmse_theta = sqrt(1/length(time) * acc_loss_theta);

    % Error plot
    plot(time, abs(e_theta)*10^3, 'LineWidth', 1.5);
    hold on;

    % Store legend entry
    legend_entries{i} = ['d\_e = ', num2str(d_e_mag), ' sin(t)'];
end

hold off;
grid on;
legend(legend_entries, 'Location', 'northeast');

% ylim([0 2]);
xlabel('time (s)');
ylabel('position error (mrad)');
title('Position error, |e_\theta|');

saveas(h1, 'Matlab plots\DiffTune_plus\Testing P-STSMC DiffTune_plus.png');

%% Duttelut

d_e_mag = 0.85;

% Simulink simulation - STSMC and P-STSMC hand-tuning
driveTrain_sim = sim('driveTrain_P_STSMC', 10);

% Extracting data
omega_r_timeseries = driveTrain_sim.omega_r_out;
theta_r_timeseries = driveTrain_sim.theta_r_out;
omega_m_timeseries = driveTrain_sim.omega_m_out;
theta_l_timeseries = driveTrain_sim.theta_l_out;

% Extracting data not as timeseries
time = omega_r_timeseries.Time;
omega_r = omega_r_timeseries.Data;
theta_r = theta_r_timeseries.Data;
omega_m = omega_m_timeseries.Data;
theta_l = theta_l_timeseries.Data;

% Loss and RSME calculations (same as used for DiffTune)
e_theta = theta_r - theta_l;
loss_theta = e_theta .^ 2;
acc_loss_theta = sum(loss_theta);   % accumulated loss
rmse_theta = sqrt(1/length(time) * acc_loss_theta);

h2 = figure(2);

subplot(2,2,[1 2]);
plot(driveTrain_sim.theta_l_out, 'LineWidth', 1.5);
hold on;
plot(driveTrain_sim.theta_r_out, '--', 'LineWidth', 1.5);
hold off;
grid on;
legend('\theta_l', '\theta_r', 'Location', 'southeast');
ylim([-1 1]);
xlabel('time (s)');
ylabel('position (rad)');
text(0.5,-0.1,['k1 = ' sprintf('%.3f', k1)]);
text(0.5,-0.3,['k2 = ' sprintf('%.3f', k2)]);
text(0.5,-0.5,['k_pos = ' sprintf('%.2f', k_pos)]);
text(0.5,-0.7,['rmse = ' sprintf('%.4f', rmse_theta)]);
title('DiffTune+ auto-tuned P-STSMC sine response');

subplot(2,2,3);
plot(time, abs(e_theta)*10^3, 'LineWidth', 1.5);
grid on;
legend('|e_\theta|', 'Location', 'northeast');
xlabel('time (s)');
ylabel('position error (mrad)');
title('Position error');

subplot(2,2,4);
plot(driveTrain_sim.T_m_out, 'LineWidth', 1.5);
hold on;
yline(2.7,'--');
hold off;
grid on;
legend('T_m = u + d_e', 'rated torque', 'Location', 'northeast');
xlabel('time (s)');
ylabel('torque (N m)');
text(4.5,1,'d_e = 0.85 sin(t)');
title('Motor torque');

% saveas(h2, 'Matlab plots\DiffTune_plus\P-STSMC DiffTune_plus check.png');
saveas(h2, 'Matlab plots\DiffTune_plus\Testing P-STSMC DiffTune_plus 0.85.png');



