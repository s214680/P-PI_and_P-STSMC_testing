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

%% Simulations - Hand-tuning

% Controller gains
% From hand-tuning
k1 = 0.492;
k2 = 10;
k_pos = 5;

% Normal load
J_l = J_l_orig;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries = driveTrain_sim.theta_r_out;
theta_l_timeseries = driveTrain_sim.theta_l_out;
u_timeseries = driveTrain_sim.u_out;
time = theta_r_timeseries.Time;
e_theta = theta_r_timeseries.Data - theta_l_timeseries.Data;

% Low load (-50 %)
J_l = J_l_l50;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_l50 = driveTrain_sim.theta_r_out;
theta_l_timeseries_l50 = driveTrain_sim.theta_l_out;
u_timeseries_l50 = driveTrain_sim.u_out;
time_l50 = theta_r_timeseries_l50.Time;
e_theta_l50 = theta_r_timeseries_l50.Data - theta_l_timeseries_l50.Data;

% High load (+50 %)
J_l = J_l_h50;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_h50 = driveTrain_sim.theta_r_out;
theta_l_timeseries_h50 = driveTrain_sim.theta_l_out;
u_timeseries_h50 = driveTrain_sim.u_out;
time_h50 = theta_r_timeseries_h50.Time;
e_theta_h50 = theta_r_timeseries_h50.Data - theta_l_timeseries_h50.Data;

% Low load (-95 %)
J_l = J_l_l95;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_l95 = driveTrain_sim.theta_r_out;
theta_l_timeseries_l95 = driveTrain_sim.theta_l_out;
u_timeseries_l95 = driveTrain_sim.u_out;
time_l95 = theta_r_timeseries_l95.Time;
e_theta_l95 = theta_r_timeseries_l95.Data - theta_l_timeseries_l95.Data;

% High load (+95 %)
J_l = J_l_h95;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_h95 = driveTrain_sim.theta_r_out;
theta_l_timeseries_h95 = driveTrain_sim.theta_l_out;
u_timeseries_h95 = driveTrain_sim.u_out;
time_h95 = theta_r_timeseries_h95.Time;
e_theta_h95 = theta_r_timeseries_h95.Data - theta_l_timeseries_h95.Data;

%% Subplot - Hand-tuning

h1 = figure(1);

set(gcf, 'Position', [100, 100, 800, 400]);

subplot(1,3,1);
plot(time_h50, abs(e_theta_h50)*10^3, '-', 'DisplayName', '+50% J_l', 'LineWidth', 1.5);
hold on;
plot(time_h95, abs(e_theta_h95)*10^3, '-', 'DisplayName', '+95% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta)*10^3, 'DisplayName', 'J_l = 8.31e-4', 'LineWidth', 1.5);
hold on;
plot(time_l50, abs(e_theta_l50)*10^3, '-', 'DisplayName', '-50% J_l', 'LineWidth', 1.5);
hold on;
plot(time_l95, abs(e_theta_l95)*10^3, '-', 'DisplayName', '-95% J_l', 'LineWidth', 1.5);
hold off;
grid on;
xlim([0 0.2]);
ylim([0 9]);
% legend;
xlabel('time (s)');
ylabel('position error (mrad)');
title('Hand-tuned P-STSMC:');


%% Simulations - DiffTune

% Controller gains
% From DiffTune
k1 = 1.779;
k2 = 0.9926;
k_pos = 4.213;

% Normal load
J_l = J_l_orig;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries = driveTrain_sim.theta_r_out;
theta_l_timeseries = driveTrain_sim.theta_l_out;
u_timeseries = driveTrain_sim.u_out;
time = theta_r_timeseries.Time;
e_theta = theta_r_timeseries.Data - theta_l_timeseries.Data;

% Low load (-50 %)
J_l = J_l_l50;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_l50 = driveTrain_sim.theta_r_out;
theta_l_timeseries_l50 = driveTrain_sim.theta_l_out;
u_timeseries_l50 = driveTrain_sim.u_out;
time_l50 = theta_r_timeseries_l50.Time;
e_theta_l50 = theta_r_timeseries_l50.Data - theta_l_timeseries_l50.Data;

% High load (+50 %)
J_l = J_l_h50;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_h50 = driveTrain_sim.theta_r_out;
theta_l_timeseries_h50 = driveTrain_sim.theta_l_out;
u_timeseries_h50 = driveTrain_sim.u_out;
time_h50 = theta_r_timeseries_h50.Time;
e_theta_h50 = theta_r_timeseries_h50.Data - theta_l_timeseries_h50.Data;

% Low load (-95 %)
J_l = J_l_l95;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_l95 = driveTrain_sim.theta_r_out;
theta_l_timeseries_l95 = driveTrain_sim.theta_l_out;
u_timeseries_l95 = driveTrain_sim.u_out;
time_l95 = theta_r_timeseries_l95.Time;
e_theta_l95 = theta_r_timeseries_l95.Data - theta_l_timeseries_l95.Data;

% High load (+95 %)
J_l = J_l_h95;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_h95 = driveTrain_sim.theta_r_out;
theta_l_timeseries_h95 = driveTrain_sim.theta_l_out;
u_timeseries_h95 = driveTrain_sim.u_out;
time_h95 = theta_r_timeseries_h95.Time;
e_theta_h95 = theta_r_timeseries_h95.Data - theta_l_timeseries_h95.Data;


%% Subplot - DiffTune

subplot(1,3,2);
plot(time_h50, abs(e_theta_h50)*10^3, '-', 'DisplayName', '+50% J_l', 'LineWidth', 1.5);
hold on;
plot(time_h95, abs(e_theta_h95)*10^3, '-', 'DisplayName', '+95% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta)*10^3, 'DisplayName', 'J_l = 8.31e-4', 'LineWidth', 1.5);
hold on;
plot(time_l50, abs(e_theta_l50)*10^3, '-', 'DisplayName', '-50% J_l', 'LineWidth', 1.5);
hold on;
plot(time_l95, abs(e_theta_l95)*10^3, '-', 'DisplayName', '-95% J_l', 'LineWidth', 1.5);
hold off;
grid on;
xlim([0 0.2]);
ylim([0 9]);
% legend;
xlabel('time (s)');
ylabel('position error (mrad)');
title('DiffTune tuned P-STSMC');

%% Simulations - DiffTune+

% Controller gains
% From DiffTune
k1 = 2.433;
k2 = 0.965;
k_pos = 11.86;

% Normal load
J_l = J_l_orig;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries = driveTrain_sim.theta_r_out;
theta_l_timeseries = driveTrain_sim.theta_l_out;
u_timeseries = driveTrain_sim.u_out;
time = theta_r_timeseries.Time;
e_theta = theta_r_timeseries.Data - theta_l_timeseries.Data;

% Low load (-50 %)
J_l = J_l_l50;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_l50 = driveTrain_sim.theta_r_out;
theta_l_timeseries_l50 = driveTrain_sim.theta_l_out;
u_timeseries_l50 = driveTrain_sim.u_out;
time_l50 = theta_r_timeseries_l50.Time;
e_theta_l50 = theta_r_timeseries_l50.Data - theta_l_timeseries_l50.Data;

% High load (+50 %)
J_l = J_l_h50;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_h50 = driveTrain_sim.theta_r_out;
theta_l_timeseries_h50 = driveTrain_sim.theta_l_out;
u_timeseries_h50 = driveTrain_sim.u_out;
time_h50 = theta_r_timeseries_h50.Time;
e_theta_h50 = theta_r_timeseries_h50.Data - theta_l_timeseries_h50.Data;

% Low load (-95 %)
J_l = J_l_l95;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_l95 = driveTrain_sim.theta_r_out;
theta_l_timeseries_l95 = driveTrain_sim.theta_l_out;
u_timeseries_l95 = driveTrain_sim.u_out;
time_l95 = theta_r_timeseries_l95.Time;
e_theta_l95 = theta_r_timeseries_l95.Data - theta_l_timeseries_l95.Data;

% High load (+95 %)
J_l = J_l_h95;
driveTrain_sim = sim('driveTrain_P_STSMC_test2', 10);

theta_r_timeseries_h95 = driveTrain_sim.theta_r_out;
theta_l_timeseries_h95 = driveTrain_sim.theta_l_out;
u_timeseries_h95 = driveTrain_sim.u_out;
time_h95 = theta_r_timeseries_h95.Time;
e_theta_h95 = theta_r_timeseries_h95.Data - theta_l_timeseries_h95.Data;

%% Subplot - DiffTune+

subplot(1,3,3);
plot(time_h50, abs(e_theta_h50)*10^3, '-', 'DisplayName', '+50% J_l', 'LineWidth', 1.5);
hold on;
plot(time_h95, abs(e_theta_h95)*10^3, '-', 'DisplayName', '+95% J_l', 'LineWidth', 1.5);
hold on;
plot(time, abs(e_theta)*10^3, 'DisplayName', 'J_l = 8.31e-4', 'LineWidth', 1.5);
hold on;
plot(time_l50, abs(e_theta_l50)*10^3, '-', 'DisplayName', '-50% J_l', 'LineWidth', 1.5);
hold on;
plot(time_l95, abs(e_theta_l95)*10^3, '-', 'DisplayName', '-95% J_l', 'LineWidth', 1.5);
hold off;
grid on;
xlim([0 0.2]);
ylim([0 9]);
legend;
xlabel('time (s)');
ylabel('position error (mrad)');
title('DiffTune+ tuned P-STSMC');

sgtitle('Position error, |e_\theta|', 'Interpreter', 'tex');


%% Save plot

saveas(h1, 'Plots\Test2_P-STSMC_position error.png');

