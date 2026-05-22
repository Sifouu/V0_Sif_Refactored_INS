% test_broad_dataset.m
% Test script to benchmark attitude observers against the BROAD dataset.

clear; clc; close all;

% Get the absolute path of the directory containing this script
script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');

addpath(genpath(fullfile(project_root, 'src')));
addpath(genpath(fullfile(project_root, 'Code - Scalar SO3 observer')));

proc_dir = fullfile(project_root, 'datasets', 'BROAD', 'processed');
files_to_test = {
    '01_undisturbed_slow_rotation_A.mat', 12000, 20000;
    '02_undisturbed_slow_rotation_B.mat', 12000, 20000;
    '03_undisturbed_slow_rotation_C.mat', 12000, 20000
};

dt = 1/286;

for i = 1:size(files_to_test, 1)
    file_name = files_to_test{i, 1};
    start_idx = files_to_test{i, 2};
    end_idx   = files_to_test{i, 3};
    
    fprintf('=== Testing %s ===\n', file_name);
    
    % Load the dataset
    data = load(fullfile(proc_dir, file_name));
    
    imu_gyr  = data.imu_gyr(start_idx:end_idx, :);
    imu_acc  = data.imu_acc(start_idx:end_idx, :);
    imu_mag  = data.imu_mag(start_idx:end_idx, :);
    opt_quat = data.opt_quat(start_idx:end_idx, :);
    
    N = size(imu_gyr, 1);
    t = (0:N-1) * dt;
    
    % Normalize vectors
    for k = 1:N
        imu_acc(k,:) = imu_acc(k,:) / norm(imu_acc(k,:));
        imu_mag(k,:) = imu_mag(k,:) / norm(imu_mag(k,:));
    end
    
    %% Setup Measurements
    measurements.imu.time = t;
    measurements.imu.w_B  = imu_gyr';
    measurements.imu.a_B  = -imu_acc'; % Negated to match expected convention: a_B = -R^T g_ref
    
    measurements.mag.time = t;
    measurements.mag.mag_meas = imu_mag';
    
    %% Compute Ground Truth and Init State
    R_gt = quat2rotm(opt_quat);
    
    % Initial estimate R^(0) with a rotation error of pi/4 about each axis
    R_err_init = ALLFUNCS.Rexp([pi/4; pi/4; pi/4]);
    R_init = R_gt(:,:,1) * R_err_init;
    init_state.R = R_init;
    
    %% Setup Common Params
    % Sensor noise standard deviations from hardware specs (myon aktos-t)
    sigma_gyro = deg2rad([0.10; 0.09; 0.12]); % Convert deg/s to rad/s
    sigma_acc  = [0.044; 0.050; 0.074] / 9.81; % Normalize by g (since a_B is normalized)
    
    % Magnetometer base magnitude
    m_ref_raw = [-0.3680; 13.6868; -39.2062];
    mag_norm = norm(m_ref_raw);
    
    sigma_mag  = [0.71; 0.70; 0.68] / mag_norm; % Normalize by baseline mag field
    
    % Construct precise covariance matrices
    params.Q_gyro = diag(sigma_gyro.^2) ;
    params.R_acc  = diag(sigma_acc.^2);
    params.R_mag  = diag(sigma_mag.^2);
    
    params.acc_axes = [1; 2; 3];
    params.mag_axes = [1; 2; 3];
    params.g_ref = [0; 0; 1];
    params.m_ref = m_ref_raw / mag_norm;
    
    %% Setup LCSS and LCSS Mahony Params
    params_lcss = params;
    params_lcss.P0 = 0.8 * eye(9);
    params_lcss.eps_reg = 0.005; 
    params_lcss.k_att = 10; % Only used by Mahony
    
    %% Setup MEKF Params
    params_mekf = params;
    params_mekf.P0 = 0.01 * eye(3);
    
    %% Run Observers
    fprintf('  Running LCSS Attitude Observer...\n');
    tic;
    est_lcss = lcss_attitude_observer(measurements, init_state, params_lcss);
    time_lcss = toc;
    
    fprintf('  Running LCSS Mahony Attitude Observer...\n');
    tic;
    est_lcss_mahony = lcss_mahony_attitude_observer(measurements, init_state, params_lcss);
    time_lcss_mahony = toc;
    
    fprintf('  Running SO3 Riccati MEKF...\n');
    tic;
    est_mekf = mekf_scalar_attitude_observer(measurements, init_state, params_mekf);
    time_mekf = toc;
    
    %% Compute Errors
    err_lcss        = zeros(N-1, 1);
    err_lcss_mahony = zeros(N-1, 1);
    err_mekf        = zeros(N-1, 1);
    
    for k = 1:N-1
        trace_lcss        = max(-1, min(1, (trace(R_gt(:,:,k)' * est_lcss.R{k}) - 1) / 2));
        trace_lcss_mahony = max(-1, min(1, (trace(R_gt(:,:,k)' * est_lcss_mahony.R{k}) - 1) / 2));
        trace_mekf        = max(-1, min(1, (trace(R_gt(:,:,k)' * est_mekf.R{k}) - 1) / 2));
        
        err_lcss(k)        = acos(trace_lcss);
        err_lcss_mahony(k) = acos(trace_lcss_mahony);
        err_mekf(k)        = acos(trace_mekf);
    end
    
    rmse_lcss        = sqrt(mean(rad2deg(err_lcss).^2));
    rmse_lcss_mahony = sqrt(mean(rad2deg(err_lcss_mahony).^2));
    rmse_mekf        = sqrt(mean(rad2deg(err_mekf).^2));
    
    fprintf('  -> LCSS Theta RMSE        : %6.3f deg  |  Time: %.3f s\n', rmse_lcss, time_lcss);
    fprintf('  -> LCSS Mahony Theta RMSE : %6.3f deg  |  Time: %.3f s\n', rmse_lcss_mahony, time_lcss_mahony);
    fprintf('  -> SO3 Riccati MEKF RMSE  : %6.3f deg  |  Time: %.3f s\n\n', rmse_mekf, time_mekf);
    
    %% Plotting
    if i == size(files_to_test, 1) || true % Plot all if desired, or just the last one
        figure('Name', sprintf('Results for %s', file_name), 'Position', [100, 100, 800, 500]);
        plot(t(1:end-1), rad2deg(err_lcss), 'r-', 'LineWidth', 1.5); hold on; grid on;
        plot(t(1:end-1), rad2deg(err_lcss_mahony), 'b-', 'LineWidth', 1.5);
        plot(t(1:end-1), rad2deg(err_mekf), 'g-', 'LineWidth', 1.5);
        
        ylabel('\theta (deg)', 'Interpreter', 'tex', 'FontSize', 16);
        xlabel('Time (s)', 'Interpreter', 'tex', 'FontSize', 16);
        title(sprintf('Observer Error Comparison - %s', file_name), 'Interpreter', 'none', 'FontSize', 14);
        legend('LCSS', 'LCSS Mahony', 'SO3 Riccati MEKF', 'Location', 'best', 'FontSize', 12);
    end
end
