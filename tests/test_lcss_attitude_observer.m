% test_lcss_attitude_observer.m
% Tests the lcss_attitude_observer with 4 different scalar measurement cases.
% Plots the attitude error  trace(I - R_true * R_hat')  for each case.
%
% Measurement cases (matching the LCSS 2025 paper / A003_pose_estimation.m):
%   Case 1 - Full vector: 3 accel rows  + 3 mag rows
%   Case 2 - Full vector: same as Case 1 (paper baseline)
%   Case 3 - Partial:     2 accel rows  + 1 mag row
%   Case 4 - Scalar:      1 accel row   + 2 mag rows

clear; clc; close all;
addpath(genpath('../src'));

fprintf('=============================================================\n');
fprintf('  LCSS Attitude Observer — 4-Case Benchmark\n');
fprintf('=============================================================\n\n');

%% 1. Ground Truth
fprintf('[1/5] Generating ground truth (1000 Hz, 3 s)...\n');
frequency_gt = 1000;
t_final      = 3;
type         = 'figure8';
perturbation = struct();

[R_true, P_true, V_true, W_true, A_true] = ...
    generate_trajectory(frequency_gt, t_final, type, perturbation);
time_gt = 0:(1/frequency_gt):t_final;

%% 2. Simulate Sensors
fprintf('[2/5] Simulating IMU and magnetometer...\n');

sigma_gyro_val = sqrt(0.001);
sigma_acc_val  = sqrt(10.0); % Increased to account for highly dynamic linear acceleration!
sigma_mag_val  = sqrt(0.1);

% --- IMU ---
imu_params.frequency       = 1000;
imu_params.gyro_noise_std  = sigma_gyro_val;
imu_params.accel_noise_std = sigma_acc_val;
imu_params.accel_bias      = zeros(3,1);
imu_params.gyro_bias       = zeros(3,1);
imu_params.g               = 9.81;
[a_B_imu, w_imu, time_imu] = simulate_imu(A_true, W_true, R_true, time_gt, imu_params);

% Build gravity-in-body measurement  g_B = -R^T [0;0;g]
% (The LCSS observer uses g_B, not the raw specific force)
N_imu = numel(time_imu);
g_B_imu = zeros(3, N_imu);
for k = 1:N_imu
    [~, idx]     = min(abs(time_gt - time_imu(k)));
    g_B_imu(:,k) = -R_true{idx}' * [0; 0; 9.81] + ...
                    sigma_acc_val * randn(3, 1);
end

% --- Magnetometer ---
mag_params.frequency = 100;
mag_params.noise_std = sigma_mag_val;
[mag_meas, time_mag] = simulate_mag(R_true, time_gt, mag_params);

% --- Pack into measurements struct ---
measurements.imu.time     = time_imu;
measurements.imu.w_B      = w_imu;
measurements.imu.a_B      = g_B_imu;    % gravity in body frame
measurements.mag.time     = time_mag;
measurements.mag.mag_meas = mag_meas;

%% 3. Reference vectors  (must match A001_generate_ground_truth_data.m)
g_ref = [0; 0; 9.81];
m_ref = [1/sqrt(2); 0; 1/sqrt(2)];   % inertial magnetic reference

%% 4. Initial state with attitude error (~22.5 deg)
% fprintf('[3/5] Setting initial state with attitude error...\n');
% rng(42);  % reproducible
% rad_std    = deg2rad(100);
% err_angles = rad_std * randn(3, 1);
% 
% Rx = @(r) [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
% Ry = @(p) [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
% Rz = @(y) [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
% 
% R_err    = Rz(err_angles(3)) * Ry(err_angles(2)) * Rx(err_angles(1));
% [U,~,V]  = svd(R_err);
% R_err    = U * diag([1, 1, det(U*V')]) * V';
% 
% % Initial estimate = first ground-truth R  * random error
% [~, idx0]    = min(abs(time_gt - time_imu(1)));
init_state.R = eye(3); %R_true{idx0} * R_err';

%% 5. Define 4 measurement cases and run observer
fprintf('[4/5] Running 4 observer cases...\n');

cases(1).label    = 'Case 1: Six (acc $x,y,z$ + mag $x,y,z$)';
cases(1).acc_axes = [1; 2; 3];
cases(1).mag_axes = [1; 2; 3];

cases(2).label    = 'Case 2: Four (acc $y,z$ + mag $x,y$)';
cases(2).acc_axes = [2; 3];
cases(2).mag_axes = [1; 2];

cases(3).label    = 'Case 3: Three (acc $y,z$ + mag $y$)';
cases(3).acc_axes = [2; 3];
cases(3).mag_axes = 2;

cases(4).label    = 'Case 4: Two (acc $y$ + mag $y$)';
cases(4).acc_axes = 2;
cases(4).mag_axes = 2;

% Plot style per case: Cases 1 and 2 are identical (same C matrices),
% so they overlap. Case 2 is shown dashed to confirm the overlap.
plot_colors  = {'b',   'b',    'r',     'm'    };
plot_styles  = {'-',   '--',   '-',     '-'    };
plot_widths  = { 2,     1.5,    1.5,     1.5   };
Error_attitude = cell(1, 4);

figure(1); clf; hold on; grid on;

for c = 1:4
    params.P0         = 0.001 * eye(9);
    params.Q_gyro     = diag(repmat(sigma_gyro_val^2, 3, 1));
    params.g_ref      = g_ref;
    params.m_ref      = m_ref;
    params.R_acc      = diag(repmat(sigma_acc_val^2, 3, 1));
    params.acc_axes   = cases(c).acc_axes;
    params.R_mag      = diag(repmat(sigma_mag_val^2, 3, 1));
    params.mag_axes   = cases(c).mag_axes;

    est = lcss_attitude_observer(measurements, init_state, params);

    % --- Build ground-truth struct downsampled to IMU rate ---
    N_est = numel(est.time);
    gt_ds.P = zeros(3, N_est);       % attitude-only: dummy zeros
    gt_ds.V = zeros(3, N_est);       % attitude-only: dummy zeros
    gt_ds.R = cell(1, N_est);
    for k = 1:N_est
        [~, idx]  = min(abs(time_gt - est.time(k)));
        gt_ds.R{k} = R_true{idx};
    end

    % --- Pad estimates with dummy .P and .V for compute_metrics ---
    est.P = zeros(3, N_est);
    est.V = zeros(3, N_est);

    % --- Compute errors via evaluation function ---
    [errors, rmse] = compute_metrics(gt_ds, est);
    Error_attitude{c} = errors.att_trace;   % 0.5 * trace(I - R_true'*R_hat)

    plot(est.time, errors.att_trace, ...
         'Color',     plot_colors{c}, ...
         'LineStyle', plot_styles{c}, ...
         'LineWidth', plot_widths{c}, ...
         'DisplayName', cases(c).label);

    fprintf('  %s\n    Initial error: %.4f  |  Final error: %.4f  |  RMSE: %.4f\n', ...
            cases(c).label, errors.att_trace(1), errors.att_trace(end), rmse.attitude_trace);
end

%% 6. Format figure
fprintf('[5/5] Saving figure...\n');

ylabel('$\mathrm{trace}(I_3 - R\hat{R}^{\top})$', ...
       'Interpreter','latex', 'FontSize', 13);
xlabel('Time (s)', 'Interpreter','latex');
legend('show', 'Interpreter','latex', 'FontSize', 11, 'Location','best');
title('LCSS Attitude Observer -- Scalar Measurement Cases', ...
      'Interpreter','latex');

