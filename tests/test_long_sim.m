% test_long_sim.m
clear; clc;
addpath(genpath('../src'));

frequency_gt = 1000;
t_final      = 15; % RUN FOR 15 SECONDS!
type         = 'figure8';
perturbation = struct();

[R_true, P_true, V_true, W_true, A_true] = generate_trajectory(frequency_gt, t_final, type, perturbation);
time_gt = 0:(1/frequency_gt):t_final;
sigma_gyro_val = sqrt(0.001);
sigma_acc_val  = sqrt(10.0); % Increased to account for highly dynamic linear acceleration!
sigma_mag_val  = sqrt(0.1);

imu_params.frequency       = 1000;
imu_params.gyro_noise_std  = sigma_gyro_val;
imu_params.accel_noise_std = sigma_acc_val;
imu_params.accel_bias      = zeros(3,1);
imu_params.gyro_bias       = zeros(3,1);
imu_params.g               = 9.81;
[a_B_imu, w_imu, time_imu] = simulate_imu(A_true, W_true, R_true, time_gt, imu_params);

mag_params.frequency = 100;
mag_params.noise_std = sigma_mag_val;
[mag_meas, time_mag] = simulate_mag(R_true, time_gt, mag_params);
a_B_imu_norm = a_B_imu ./ vecnorm(a_B_imu, 2, 1);
mag_meas_norm = mag_meas ./ vecnorm(mag_meas, 2, 1);

measurements.imu.time     = time_imu;
measurements.imu.w_B      = w_imu;
measurements.imu.a_B      = -a_B_imu_norm;   
measurements.mag.time     = time_mag;
measurements.mag.mag_meas = mag_meas_norm;

g_ref = [0; 0; 1];
m_ref = [1/sqrt(2); 0; 1/sqrt(2)];

init_state.R = eye(3);

dt = 1 / frequency_gt;
params_mahony.P0         = 0.001 * eye(9);
params_mahony.Q_gyro     = diag(repmat(sigma_gyro_val^2, 3, 1));
params_mahony.g_ref      = g_ref;
params_mahony.m_ref      = m_ref;
params_mahony.R_acc      = diag(repmat(sigma_acc_val^2, 3, 1));
params_mahony.acc_axes   = [1;2;3];
params_mahony.R_mag      = diag(repmat(sigma_mag_val^2, 3, 1));
params_mahony.mag_axes   = [1;2;3];
params_mahony.k_att      = 10.0;

est_mahony = lcss_mahony_attitude_observer(measurements, init_state, params_mahony);

N_est = numel(time_imu);
gt_ds.P = zeros(3, N_est);
gt_ds.V = zeros(3, N_est);
gt_ds.R = cell(1, N_est);
for k = 1:N_est
    [~, idx]  = min(abs(time_gt - time_imu(k)));
    gt_ds.R{k} = R_true{idx};
end

est_mahony.P = zeros(3, N_est); est_mahony.V = zeros(3, N_est);
[err_mahony, rmse_mahony] = compute_metrics(gt_ds, est_mahony);

fprintf('Long Sim (15s) Final Error: %.4f\n', err_mahony.att_trace(end));
