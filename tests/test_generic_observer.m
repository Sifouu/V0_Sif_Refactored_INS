% test_generic_observer.m
% Script to verify the generic observer with modular sensor data

clear; clc; close all;
addpath(genpath('../src'));  % All source modules (sensors, observers, evaluation, utils, animation)

%% 1. Generate High-Frequency Ground Truth
frequency_gt = 1000;
t_final = 20;
type = 'figure8';
perturbation = struct();
disp('Generating ground truth (1000 Hz)...');
[R_true, P_true, V_true, W_true, A_true] = generate_trajectory(frequency_gt, t_final, type, perturbation);
time_gt = 0:(1/frequency_gt):t_final;

ground_truth.P = P_true;
ground_truth.V = V_true;
ground_truth.R = R_true;

%% 2. Generate Sensor Data
disp('Simulating sensors...');

% IMU (200 Hz)
imu_params.frequency = 1000;
imu_params.accel_noise_std = 0.05;
imu_params.gyro_noise_std = 0.01;
imu_params.accel_bias = zeros(3,1); % Keep bias zero for now
imu_params.gyro_bias = zeros(3,1);
imu_params.g = 9.81;
[measurements.imu.a_B, measurements.imu.w_B, measurements.imu.time] = simulate_imu(A_true, W_true, R_true, time_gt, imu_params);

% GPS (5 Hz) with outage
gps_params.frequency = 1000;
gps_params.noise_std_pos = 0.5;
gps_params.noise_std_vel = 0.1;
gps_params.outage_start = 4.0;
gps_params.outage_end = 7.0;
[measurements.gps.p_meas, measurements.gps.v_meas, measurements.gps.is_valid, measurements.gps.time] = simulate_gps(P_true, V_true, time_gt, gps_params);

% Camera/Landmarks (20 Hz)
cam_params.frequency = 1000;
cam_params.num_landmarks = 5;
cam_params.noise_std_pos = 0.1;
cam_params.noise_std_bearing = 0.01;
[measurements.cam.landmark_meas, measurements.cam.bearing, measurements.cam.positions_I, measurements.cam.time] = simulate_landmarks(P_true, R_true, time_gt, cam_params);

%% 3. Setup Observer
% Initial state with some errors
init_state.P = P_true(:, 1) + [1; -1; 0.5];
init_state.V = V_true(:, 1) + [0.2; -0.1; 0];
% Small attitude error
theta = 0.1;
R_err = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
init_state.R = R_true{1} * R_err;

% Filter tuning
params.P0 = 1 * eye(15);
params.V_noise = 100 * eye(15); % Process noise (equivalent to V in original code)
params.Q_gps = 0.05 * eye(3);
params.Q_cam = 0.05 * eye(3);
params.g = 9.81;
params.k_att = 100;

%% 4. Run Observer
disp('Running generic observer...');
tic;
estimates = generic_observer(measurements, init_state, params);
toc;

%% 5. Evaluate Metrics
disp('Computing metrics...');
% Downsample ground truth to match IMU/estimator frequency
M_est = length(estimates.time);
gt_downsampled.P = zeros(3, M_est);
gt_downsampled.V = zeros(3, M_est);
gt_downsampled.R = cell(1, M_est);

for k = 1:M_est
    [~, idx] = min(abs(time_gt - estimates.time(k)));
    gt_downsampled.P(:, k) = P_true(:, idx);
    gt_downsampled.V(:, k) = V_true(:, idx);
    gt_downsampled.R{k}    = R_true{idx};
end

[errors, rmse] = compute_metrics(gt_downsampled, estimates);

fprintf('\n--- RMSE Results ---\n');
fprintf('Position Euclidean: %.4f m\n', rmse.position_euclidean);
fprintf('Velocity Euclidean: %.4f m/s\n', rmse.velocity_euclidean);
fprintf('Attitude Trace:     %.6f\n', rmse.attitude_trace);

%% 6. Plot Results
disp('Plotting results...');
fig = plot_paper_results(ground_truth, estimates, errors, 'Stereo-aided-INS');
savefig(fig, '../figures/generic_observer_results.fig');
print(fig, '../figures/generic_observer_results', '-dpng', '-r300');

%% 7. Animate Results
disp('Generating animation...');
animate_results(gt_downsampled, estimates, errors, '../figures/generic_observer_animation.avi');

disp('Test complete.');
