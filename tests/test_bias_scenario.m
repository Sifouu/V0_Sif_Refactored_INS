% test_bias_scenario.m
% Tests the observers under a severe "industrial" gyroscope bias to demonstrate
% the native bias compensation of the Mahony cascade vs the baseline LCSS-SVD.

clear; clc; close all;
addpath(genpath('../src'));

fprintf('=============================================================\n');
fprintf('  Bias Scenario: LCSS-SVD vs LCSS-Mahony \n');
fprintf('=============================================================\n\n');

%% 1. Ground Truth
fprintf('[1/5] Generating ground truth...\n');
freq_gt = 1000;
t_final = 10; % 10 seconds to allow bias to converge
[R_true, P_true, V_true, W_true, A_true] = generate_trajectory(freq_gt, t_final, 'figure8', struct());
time_gt = 0:(1/freq_gt):t_final;

%% 2. Simulate Sensors with Bias
fprintf('[2/5] Simulating IMU with severe gyroscope bias...\n');
sigma_gyro = sqrt(.001);
sigma_acc  = sqrt(.1);
sigma_mag  = sqrt(.01);

imu_params.frequency       = 100; % 100 Hz IMU
imu_params.gyro_noise_std  = sigma_gyro;
imu_params.accel_noise_std = sigma_acc;
imu_params.accel_bias      = zeros(3,1);

% INDUSTRIAL GYRO BIAS: [5 deg/s, -8 deg/s, 10 deg/s] 
true_gyro_bias = [0.087; -0.139; 0.174];
imu_params.gyro_bias       = true_gyro_bias; 
imu_params.g               = 9.81;

[a_B_imu, w_imu, time_imu] = simulate_imu(A_true, W_true, R_true, time_gt, imu_params);

% Re-construct true gravity for observer in body frame (LCSS convention)
N_imu = numel(time_imu);
g_B_imu = zeros(3, N_imu);
for k = 1:N_imu
    [~, idx]     = min(abs(time_gt - time_imu(k)));
    g_B_imu(:,k) = -R_true{idx}' * [0; 0; 9.81] + sigma_acc * randn(3, 1);
end

mag_params.frequency = 100;
mag_params.noise_std = sigma_mag;
[mag_meas, time_mag] = simulate_mag(R_true, time_gt, mag_params);

measurements.imu.time     = time_imu;
measurements.imu.w_B      = w_imu;
measurements.imu.a_B      = g_B_imu;
measurements.mag.time     = time_mag;
measurements.mag.mag_meas = mag_meas;

%% 3. Observers Setup
params.P0 = 0.01 * eye(9);
params.Q_gyro = diag(repmat(sigma_gyro^2, 3, 1));
params.R_acc = diag(repmat(sigma_acc^2, 3, 1));
params.R_mag = diag(repmat(sigma_mag^2, 3, 1));
params.acc_axes = [1;2;3];
params.mag_axes = [1;2;3];

% Mahony Gains
params.k_att = 5.0; % Proportional gain
params.k_I   = 0.0; % Integral gain disabled

params.g_ref = [0; 0; 9.81];
params.m_ref = [1/sqrt(2); 0; 1/sqrt(2)];

init_state.R = eye(3);

%% 4. Run Observers
fprintf('[3/5] Running LCSS-SVD...\n');
est_svd = lcss_attitude_observer(measurements, init_state, params);

fprintf('[4/5] Running LCSS-Mahony (No bias compensation)...\n');
est_mahony = lcss_mahony_attitude_observer(measurements, init_state, params);

%% 5. Evaluate and Plot
fprintf('[5/5] Computing metrics and plotting...\n');
N_est = numel(time_imu);
gt_ds.P = zeros(3, N_est); gt_ds.V = zeros(3, N_est);
gt_ds.R = cell(1, N_est);
for k = 1:N_est
    [~, idx] = min(abs(time_gt - time_imu(k)));
    gt_ds.R{k} = R_true{idx};
end

est_svd.P = zeros(3, N_est); est_svd.V = zeros(3, N_est);
est_mahony.P = zeros(3, N_est); est_mahony.V = zeros(3, N_est);

[err_svd, rmse_svd] = compute_metrics(gt_ds, est_svd);
[err_mahony, rmse_mahony] = compute_metrics(gt_ds, est_mahony);

fprintf('\n=== Trace Error Metrics ===\n');
fprintf('LCSS-SVD    RMSE: %.4f | Final: %.4f\n', rmse_svd.attitude_trace, err_svd.att_trace(end));
fprintf('LCSS-Mahony RMSE: %.4f | Final: %.4f\n', rmse_mahony.attitude_trace, err_mahony.att_trace(end));

figure('Position', [100, 100, 1000, 600]);
plot(est_svd.time, err_svd.att_trace, 'b-', 'LineWidth', 2, 'DisplayName', 'LCSS-SVD');
hold on; grid on;
plot(est_mahony.time, err_mahony.att_trace, 'g-', 'LineWidth', 2, 'DisplayName', 'LCSS-Mahony (No Bias Comp)');
legend('Location', 'best', 'FontSize', 14);
title('Attitude Error Under Severe Gyroscope Bias (Industrial IMU)');
ylabel('Trace Error'); xlabel('Time (s)');
ylim([0 max(err_svd.att_trace)*1.2]);

if ~isfolder('../figures'), mkdir('../figures'); end
print(gcf, '../figures/test_bias_scenario', '-dpng', '-r300');
fprintf('Figure saved to ../figures/test_bias_scenario.png\n');
