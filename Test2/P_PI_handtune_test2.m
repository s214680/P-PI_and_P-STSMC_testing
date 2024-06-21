clear all;
clc;

addpath('Plots');

%% Model and simulation init
% Frequency for reference sine wave
xf = 1;

% Parameters for drive train
% This is the initialization script for the motor and axle parameters. Both
% motors are identical PMSM 1FT7042-5AF70-1DA0 HD.

% Motor and load mechanical parameters
N = 1;              % -- Gear ratio
J_m = 8.31e-4;      % kg m^2 -- Moment of inertia
% J_l = 8.31e-4;      % kg m^2 -- Moment of inertia

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

% Controller gains
% From hand-tuning
tau_i = 0.0373;
k_vel = 0.492;
k_pos = 5.50;

%% J_l values
J_l_orig = 8.31e-4; % -- kg m^2

% +/- 10 %
J_l_l10 = J_l_orig - (J_l_orig * 0.10);
J_l_h10 = J_l_orig + (J_l_orig * 0.10);

% +/- 50 %
J_l_l50 = J_l_orig - (J_l_orig * 0.50);
J_l_h50 = J_l_orig + (J_l_orig * 0.50);

% +/- 95 %  (100 % would result in dividing with 0)
J_l_l95 = J_l_orig - (J_l_orig * 0.95);
J_l_h95 = J_l_orig + (J_l_orig * 0.95);

%% Simulations
% Normal load
J_l = J_l_orig;
driveTrain_sim = sim('driveTrain_P_PI_test2', 10);

theta_r_timeseries = driveTrain_sim.theta_r_out;
theta_l_timeseries = driveTrain_sim.theta_l_out;
u_timeseries = driveTrain_sim.u_out;
time = theta_r_timeseries.Time;
theta_r = theta_r_timeseries.Data;
theta_l = theta_l_timeseries.Data;
e_theta = theta_r - theta_l;

% Low load (-10 %)
J_l = J_l_l10;
driveTrain_sim = sim('driveTrain_P_PI_test2', 10);

theta_r_timeseries_l10 = driveTrain_sim.theta_r_out;
theta_l_timeseries_l10 = driveTrain_sim.theta_l_out;
u_timeseries_l10 = driveTrain_sim.u_out;
time_l10 = theta_r_timeseries_l10.Time;
theta_r_l10 = theta_r_timeseries_l10.Data;
theta_l_l10 = theta_l_timeseries_l10.Data;
e_theta_l10 = theta_r_l10 - theta_l_l10;
rmse_theta_l10 = sqrt(1/length(time_l10) * sum(e_theta_l10 .^ 2));

% High load (+10 %)
J_l = J_l_h10;
driveTrain_sim = sim('driveTrain_P_PI_test2', 10);

theta_r_timeseries_h10 = driveTrain_sim.theta_r_out;
theta_l_timeseries_h10 = driveTrain_sim.theta_l_out;
u_timeseries_h10 = driveTrain_sim.u_out;
time_h10 = theta_r_timeseries_h10.Time;
theta_r_h10 = theta_r_timeseries_h10.Data;
theta_l_h10 = theta_l_timeseries_h10.Data;
e_theta_h10 = theta_r_h10 - theta_l_h10;
rmse_theta_h10 = sqrt(1/length(time_h10) * sum(e_theta_h10 .^ 2));

% Low load (-50 %)
J_l = J_l_l50;
driveTrain_sim = sim('driveTrain_P_PI_test2', 10);

theta_r_timeseries_l50 = driveTrain_sim.theta_r_out;
theta_l_timeseries_l50 = driveTrain_sim.theta_l_out;
u_timeseries_l50 = driveTrain_sim.u_out;
time_l50 = theta_r_timeseries_l50.Time;
theta_r_l50 = theta_r_timeseries_l50.Data;
theta_l_l50 = theta_l_timeseries_l50.Data;
e_theta_l50 = theta_r_l50 - theta_l_l50;
rmse_theta_l50 = sqrt(1/length(time_l50) * sum(e_theta_l50 .^ 2));

% High load (+50 %)
J_l = J_l_h50;
driveTrain_sim = sim('driveTrain_P_PI_test2', 10);

theta_r_timeseries_h50 = driveTrain_sim.theta_r_out;
theta_l_timeseries_h50 = driveTrain_sim.theta_l_out;
u_timeseries_h50 = driveTrain_sim.u_out;
time_h50 = theta_r_timeseries_h50.Time;
theta_r_h50 = theta_r_timeseries_h50.Data;
theta_l_h50 = theta_l_timeseries_h50.Data;
e_theta_h50 = theta_r_h50 - theta_l_h50;
rmse_theta_h50 = sqrt(1/length(time_h50) * sum(e_theta_h50 .^ 2));

% Low load (-95 %)
J_l = J_l_l95;
driveTrain_sim = sim('driveTrain_P_PI_test2', 10);

theta_r_timeseries_l95 = driveTrain_sim.theta_r_out;
theta_l_timeseries_l95 = driveTrain_sim.theta_l_out;
u_timeseries_l95 = driveTrain_sim.u_out;
time_l95 = theta_r_timeseries_l95.Time;
theta_r_l95 = theta_r_timeseries_l95.Data;
theta_l_l95 = theta_l_timeseries_l95.Data;
e_theta_l95 = theta_r_l95 - theta_l_l95;
rmse_theta_l95 = sqrt(1/length(time_l95) * sum(e_theta_l95 .^ 2));

% High load (+95 %)
J_l = J_l_h95;
driveTrain_sim = sim('driveTrain_P_PI_test2', 10);

theta_r_timeseries_h95 = driveTrain_sim.theta_r_out;
theta_l_timeseries_h95 = driveTrain_sim.theta_l_out;
u_timeseries_h95 = driveTrain_sim.u_out;
time_h95 = theta_r_timeseries_h95.Time;
theta_r_h95 = theta_r_timeseries_h95.Data;
theta_l_h95 = theta_l_timeseries_h95.Data;
e_theta_h95 = theta_r_h95 - theta_l_h95;
rmse_theta_h95 = sqrt(1/length(time_h95) * sum(e_theta_h95 .^ 2));

%% Plot used to see check
h1 = figure(1);

plot(time, abs(e_theta_h10)*10^3, '-', 'DisplayName', '+10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h50)*10^3, '-', 'DisplayName', '+50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h95)*10^3, '-', 'DisplayName', '+95% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta)*10^3, 'DisplayName', 'J_l = 8.31e-4', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l10)*10^3, '-', 'DisplayName', '-10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l50)*10^3, '-', 'DisplayName', '-50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l95)*10^3, '-', 'DisplayName', '-95% J_l', 'LineWidth', 1.5);
hold off;
grid on;
legend;
xlabel('time (s)');
ylabel('position error (mrad)');
title('Hand-tuned P-PI: Position error, |e_\theta|', 'Interpreter', 'tex');

%% Plotting
h2 = figure(2);

subplot(3,2,[1 2]);
plot(time, abs(e_theta_h10)*10^3, '-', 'DisplayName', '+10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h50)*10^3, '-', 'DisplayName', '+50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h95)*10^3, '-', 'DisplayName', '+95% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta)*10^3, 'DisplayName', 'J_l = 8.31e-4', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l10)*10^3, '-', 'DisplayName', '-10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l50)*10^3, '-', 'DisplayName', '-50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l95)*10^3, '-', 'DisplayName', '-95% J_l', 'LineWidth', 1.5);
hold off;
grid on;
ylim([0 6]);
xlabel('time (s)');
ylabel('position error (mrad)');
title('Hand-tuned P-PI: Position error, |e_\theta|', 'Interpreter', 'tex');

subplot(3,2,[3 5]);
plot(time, abs(e_theta)*10^3, 'DisplayName', 'J_l = 8.31e-4', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l10)*10^3, '-', 'DisplayName', '-10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l50)*10^3, '-', 'DisplayName', '-50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l95)*10^3, '-', 'DisplayName', '-95% J_l', 'LineWidth', 1.5);
hold off;
grid on;
legend;
xlim([0 0.2]);
ylim([0 10]);
xlabel('time (s)');
ylabel('position error (mrad)');

subplot(3,2,[4 6]);
plot(time, abs(e_theta)*10^3, 'DisplayName', 'J_l = 8.31e-4', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h10)*10^3, '-', 'DisplayName', '+10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h50)*10^3, '-', 'DisplayName', '+50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h95)*10^3, '-', 'DisplayName', '+95% J_l', 'LineWidth', 1.5);
hold off
grid on;
legend;
xlim([0 0.2]);
ylim([0 10]);
xlabel('time (s)');
ylabel('position error (mrad)');

saveas(h2, 'Plots\Test3_P-PI_hand-tuning_position error all.png');


%% Plotting
h3 = figure(3);

subplot(3,2,[1 3]);
plot(time, abs(e_theta_h10)*10^3, '-', 'DisplayName', '+10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h50)*10^3, '-', 'DisplayName', '+50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h95)*10^3, '-', 'DisplayName', '+95% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta)*10^3, 'DisplayName', 'J_l = 8.31e-4', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l10)*10^3, '-', 'DisplayName', '-10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l50)*10^3, '-', 'DisplayName', '-50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l95)*10^3, '-', 'DisplayName', '-95% J_l', 'LineWidth', 1.5);
hold off;
grid on;
xlim([0 0.2]);
ylim([0 10]);
legend;
% xlabel('time (s)');
ylabel('position error (mrad)');
title('Hand-tuned P-PI: Position error, |e_\theta|', 'Interpreter', 'tex');

subplot(3,2,5);
plot(time, abs(e_theta_h10)*10^3, '-', 'DisplayName', '+10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h50)*10^3, '-', 'DisplayName', '+50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h95)*10^3, '-', 'DisplayName', '+95% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta)*10^3, 'DisplayName', 'J_l = 8.31e-4', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l10)*10^3, '-', 'DisplayName', '-10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l50)*10^3, '-', 'DisplayName', '-50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l95)*10^3, '-', 'DisplayName', '-95% J_l', 'LineWidth', 1.5);
hold off;
grid on;
% ylim([0 6]);
xlabel('time (s)');
% ylabel('pos. error (mrad)');

saveas(h3, 'Plots\Test3_P-PI_hand-tuning_position error all.png');


%% Plotting
h3 = figure(3);

set(gcf, 'Position', [100, 100, 800, 400]);

subplot(1,3,1);
plot(time, abs(e_theta_h10)*10^3, '-', 'DisplayName', '+10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h50)*10^3, '-', 'DisplayName', '+50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_h95)*10^3, '-', 'DisplayName', '+95% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta)*10^3, 'DisplayName', 'J_l = 8.31e-4', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l10)*10^3, '-', 'DisplayName', '-10% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l50)*10^3, '-', 'DisplayName', '-50% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta_l95)*10^3, '-', 'DisplayName', '-95% J_l', 'LineWidth', 1.5);
hold off;
grid on;
xlim([0 0.2]);
ylim([0 10]);
legend;
xlabel('time (s)');
ylabel('position error (mrad)');
title('Hand-tuned P-PI: Position error, |e_\theta|', 'Interpreter', 'tex');

% saveas(h3, 'Plots\Test3_P-PI_hand-tuning_position error all.png');

