%% 
% compare_observers.m
% Script to compare the performance of multiple observers side-by-side.

clear; clc; close all;
addpath(genpath('../src'));  % All source modules

%% 1. Generate High-Frequency Ground Truth
frequency_gt = 1000;
t_final = 25;
type = 'figure8';
perturbation = struct();
disp('Generating ground truth (1000 Hz)...');
[R_true, P_true, V_true, W_true, A_true] = generate_trajectory(frequency_gt, t_final, type, perturbation);
time_gt = 0:(1/frequency_gt):t_final;

ground_truth.P = P_true;
ground_truth.V = V_true;
ground_truth.R = R_true;
ground_truth.time = time_gt; 

%% 2. Generate Sensor Data
disp('Simulating sensors...');

% IMU (1000 Hz)
imu_params.frequency = 1000;
imu_params.accel_noise_std = 0.05;
imu_params.gyro_noise_std = 0.01;
imu_params.accel_bias = zeros(3,1);
imu_params.gyro_bias = zeros(3,1);
imu_params.g = 9.81;
[measurements.imu.a_B, measurements.imu.w_B, measurements.imu.time] = simulate_imu(A_true, W_true, R_true, time_gt, imu_params);

% GPS (5 Hz) with 3-second outage
gps_params.frequency = 1000;
gps_params.noise_std_pos = 0.5;
gps_params.noise_std_vel = 0.1;
gps_params.outage_start = 10.0*0;
gps_params.outage_end = 13.0*0;
[measurements.gps.p_meas, measurements.gps.v_meas, measurements.gps.is_valid, measurements.gps.time] = simulate_gps(P_true, V_true, time_gt, gps_params);

% Camera/Landmarks (20 Hz)
cam_params.frequency = 1000;
cam_params.num_landmarks = 10;
cam_params.noise_std_pos = 0.1;
cam_params.noise_std_bearing = 0.01;
[measurements.cam.landmark_meas, measurements.cam.bearing, measurements.cam.positions_I, measurements.cam.time] = simulate_landmarks(P_true, R_true, time_gt, cam_params);

%% 3. Setup Common Parameters
params.g = 9.81;
params.g_vec = [0; 0; -params.g];
params.Q_gps = gps_params.noise_std_pos^2 * eye(3);
params.Q_cam = cam_params.noise_std_pos^2 * eye(3);

% P0 Initialization (Generic uses 15x15, EKF uses 9x9 but will slice it automatically)
params.P0 = blkdiag(eye(3), eye(3), 0.1*eye(3), 0.1*eye(3), 0.1*eye(3));
params.V_noise = blkdiag(0.1*eye(3), 0.1*eye(3), 0.01*eye(3), 0.01*eye(3), 0.01*eye(3));
params.k_att = 10;

% Initial State with error
init_state.P = P_true(:, 1) + [1.5; -1.5; 0.5];
init_state.V = V_true(:, 1) + [0.5; -0.5; 0];
theta = 10;
R_err = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
init_state.R = R_true{1} * R_err;

%% 4. Run Observers
% 4A. Generic Observer (LPV NCF)
disp('Running Generic Observer...');
tic;
est_generic = generic_observer(measurements, init_state, params);
time_gen = toc;
fprintf('Generic Observer completed in %.3f seconds.\n', time_gen);

% 4B. Standard EKF Observer
disp('Running Standard EKF Observer...');
tic;
est_ekf = ekf_observer(measurements, init_state, params);
time_ekf = toc;
fprintf('Standard EKF completed in %.3f seconds.\n', time_ekf);

%% 5. Compute Metrics
disp('Computing metrics...');
% Downsample Ground Truth
M_est = length(est_generic.time);
gt_downsampled.time = est_generic.time;
gt_downsampled.P = zeros(3, M_est);
gt_downsampled.V = zeros(3, M_est);
gt_downsampled.R = cell(1, M_est);
for k = 1:M_est
    [~, idx] = min(abs(time_gt - est_generic.time(k)));
    gt_downsampled.P(:, k) = P_true(:, idx);
    gt_downsampled.V(:, k) = V_true(:, idx);
    gt_downsampled.R{k}    = R_true{idx};
end

[err_generic, rmse_gen] = compute_metrics(gt_downsampled, est_generic);
[err_ekf, rmse_ekf]     = compute_metrics(gt_downsampled, est_ekf);

% Print Comparison Table
fprintf('\n=======================================================\n');
fprintf('                  RMSE Comparison Table\n');
fprintf('=======================================================\n');
fprintf('Metric               | Generic Observer | Standard EKF\n');
fprintf('-------------------------------------------------------\n');
fprintf('Position (m)         | %16.4f | %12.4f\n', rmse_gen.position_euclidean, rmse_ekf.position_euclidean);
fprintf('Velocity (m/s)       | %16.4f | %12.4f\n', rmse_gen.velocity_euclidean, rmse_ekf.velocity_euclidean);
fprintf('Attitude Trace       | %16.6f | %12.6f\n', rmse_gen.attitude_trace, rmse_ekf.attitude_trace);
fprintf('=======================================================\n');

%% 6. Plot Side-by-Side Comparison
disp('Plotting results...');
estimates_list = {est_generic, est_ekf};
errors_list    = {err_generic, err_ekf};
legend_names   = {'Generic Observer', 'Standard EKF'};

plot_paper_results(ground_truth, estimates_list, errors_list, legend_names);

disp('Comparison test complete.');
