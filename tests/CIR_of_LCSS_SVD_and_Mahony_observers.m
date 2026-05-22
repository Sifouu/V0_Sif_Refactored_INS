% CIR_of_LCSS_SVD_and_Mahony_observers.m
% Tests and compares the LCSS SVD, LCSS Mahony (Fixed), and LCSS Mahony (Dynamic).
% Evaluates 4 measurement cases and plots results in a 2x2 subplot.

clear; clc; close all;
addpath(genpath('../src'));

fprintf('=============================================================\n');
fprintf('  Observer Comparison: LCSS SVD vs LCSS Mahony (Fix & Dyn) \n');
fprintf('=============================================================\n\n');

%% 1. Ground Truth
fprintf('[1/5] Generating ground truth (1000 Hz, 20 s)...\n');
frequency_gt = 1000;
t_final      = 1;
type         = 'figure8';
perturbation = struct();

[R_true, P_true, V_true, W_true, A_true] = ...
    generate_trajectory(frequency_gt, t_final, type, perturbation);
time_gt = 0:(1/frequency_gt):t_final;

%% 2. Simulate Sensors
fprintf('[2/5] Simulating IMU and magnetometer...\n');

sigma_gyro_val = sqrt(.010);
sigma_acc_val  = sqrt(0.1); % Increased to account for highly dynamic linear acceleration!
sigma_mag_val  = sqrt(0.0100);

% --- IMU ---
imu_params.frequency       = 500;
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
mag_params.frequency = 200;
mag_params.noise_std = sigma_mag_val;
[mag_meas, time_mag] = simulate_mag(R_true, time_gt, mag_params);

% --- Pack into measurements struct ---
measurements.imu.time     = time_imu;
measurements.imu.w_B      = w_imu;
measurements.imu.a_B      = g_B_imu;    % gravity in body frame
measurements.mag.time     = time_mag;
measurements.mag.mag_meas = mag_meas;

%% 3. Reference vectors
g_ref = [0; 0; 9.81];
m_ref = [1/sqrt(2); 0; 1/sqrt(2)];

%% 4. Initial state with attitude error
fprintf('[3/5] Setting initial state...\n');
% Identity matrix yields a 90-degree error initially
init_state.R = eye(3);

%% 5. Define 4 measurement cases and run observer
fprintf('[4/5] Running observers for 4 cases...\n');

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

% --- Initialize Metrics Storage ---
metrics_lcss = struct('RMSE', zeros(1,4), 'SS_RMSE', zeros(1,4), 'MaxErr', zeros(1,4), ...
                      'ConvTime', zeros(1,4), 'ExecTime', zeros(1,4), 'StepTime', zeros(1,4));
metrics_mahony_fix = struct('RMSE', zeros(1,4), 'SS_RMSE', zeros(1,4), 'MaxErr', zeros(1,4), ...
                        'ConvTime', zeros(1,4), 'ExecTime', zeros(1,4), 'StepTime', zeros(1,4));
metrics_mahony_dyn = struct('RMSE', zeros(1,4), 'SS_RMSE', zeros(1,4), 'MaxErr', zeros(1,4), ...
                        'ConvTime', zeros(1,4), 'ExecTime', zeros(1,4), 'StepTime', zeros(1,4));

% --- Build ground-truth struct downsampled to IMU rate ---
N_est = numel(time_imu);
gt_ds.P = zeros(3, N_est);
gt_ds.V = zeros(3, N_est);
gt_ds.R = cell(1, N_est);
for k = 1:N_est
    [~, idx]  = min(abs(time_gt - time_imu(k)));
    gt_ds.R{k} = R_true{idx};
end

figure(1); clf;
set(gcf, 'Position', [100, 100, 1200, 800]); 

for c = 1:4
    dt = 1 / frequency_gt;
    
    % LCSS Parameters
    params_lcss.P0         = 0.01 * eye(9);
    params_lcss.Q_gyro     = diag(repmat(sigma_gyro_val^2, 3, 1));
    params_lcss.g_ref      = g_ref;
    params_lcss.m_ref      = m_ref;
    params_lcss.R_acc      = diag(repmat(sigma_acc_val^2, 3, 1));
    params_lcss.acc_axes   = cases(c).acc_axes;
    params_lcss.R_mag      = diag(repmat(sigma_mag_val^2, 3, 1));
    params_lcss.mag_axes   = cases(c).mag_axes;
    
    % LCSS Mahony Fixed Parameters
    params_mahony_fix = params_lcss;
    params_mahony_fix.k_att = 15; % NCF gain

    % LCSS Mahony Dynamic Parameters
    params_mahony_dyn = params_lcss;
    params_mahony_dyn.dynamic_alpha = 10;
    params_mahony_dyn.k_min = 15.0;

    % Run Observers
    tic;
    est_lcss   = lcss_attitude_observer(measurements, init_state, params_lcss);
    time_lcss = toc;
    
    tic;
    est_mahony_fix = lcss_mahony_attitude_observer(measurements, init_state, params_mahony_fix);
    time_mahony_fix = toc;

    tic;
    est_mahony_dyn = lcss_mahony_dynamic_attitude_observer(measurements, init_state, params_mahony_dyn);
    time_mahony_dyn = toc;

    % --- Pad estimates with dummy .P and .V ---
    est_lcss.P = zeros(3, N_est); est_lcss.V = zeros(3, N_est);
    est_mahony_fix.P = zeros(3, N_est); est_mahony_fix.V = zeros(3, N_est);
    est_mahony_dyn.P = zeros(3, N_est); est_mahony_dyn.V = zeros(3, N_est);

    % --- Compute errors ---
    [err_lcss, rmse_lcss] = compute_metrics(gt_ds, est_lcss);
    [err_mahony_fix, rmse_mahony_fix] = compute_metrics(gt_ds, est_mahony_fix);
    [err_mahony_dyn, rmse_mahony_dyn] = compute_metrics(gt_ds, est_mahony_dyn);

    % --- Extract Metrics ---
    conv_threshold = 0.05;
    
    % LCSS SVD
    metrics_lcss.RMSE(c) = rmse_lcss.attitude_trace;
    metrics_lcss.SS_RMSE(c) = sqrt(mean(err_lcss.att_trace(round(end/2):end).^2));
    metrics_lcss.MaxErr(c) = max(err_lcss.att_trace);
    metrics_lcss.ExecTime(c) = time_lcss * 1000; % ms
    metrics_lcss.StepTime(c) = time_lcss / N_est * 1e6; % us
    idx_conv = find(err_lcss.att_trace > conv_threshold, 1, 'last');
    if isempty(idx_conv), metrics_lcss.ConvTime(c) = 0;
    elseif idx_conv == N_est, metrics_lcss.ConvTime(c) = NaN;
    else metrics_lcss.ConvTime(c) = est_lcss.time(idx_conv); end
    
    % LCSS Mahony Fixed
    metrics_mahony_fix.RMSE(c) = rmse_mahony_fix.attitude_trace;
    metrics_mahony_fix.SS_RMSE(c) = sqrt(mean(err_mahony_fix.att_trace(round(end/2):end).^2));
    metrics_mahony_fix.MaxErr(c) = max(err_mahony_fix.att_trace);
    metrics_mahony_fix.ExecTime(c) = time_mahony_fix * 1000; % ms
    metrics_mahony_fix.StepTime(c) = time_mahony_fix / N_est * 1e6; % us
    idx_conv = find(err_mahony_fix.att_trace > conv_threshold, 1, 'last');
    if isempty(idx_conv), metrics_mahony_fix.ConvTime(c) = 0;
    elseif idx_conv == N_est, metrics_mahony_fix.ConvTime(c) = NaN;
    else metrics_mahony_fix.ConvTime(c) = est_mahony_fix.time(idx_conv); end

    % LCSS Mahony Dynamic
    metrics_mahony_dyn.RMSE(c) = rmse_mahony_dyn.attitude_trace;
    metrics_mahony_dyn.SS_RMSE(c) = sqrt(mean(err_mahony_dyn.att_trace(round(end/2):end).^2));
    metrics_mahony_dyn.MaxErr(c) = max(err_mahony_dyn.att_trace);
    metrics_mahony_dyn.ExecTime(c) = time_mahony_dyn * 1000; % ms
    metrics_mahony_dyn.StepTime(c) = time_mahony_dyn / N_est * 1e6; % us
    idx_conv = find(err_mahony_dyn.att_trace > conv_threshold, 1, 'last');
    if isempty(idx_conv), metrics_mahony_dyn.ConvTime(c) = 0;
    elseif idx_conv == N_est, metrics_mahony_dyn.ConvTime(c) = NaN;
    else metrics_mahony_dyn.ConvTime(c) = est_mahony_dyn.time(idx_conv); end

    % --- Plotting in 2x2 Grid ---
    subplot(2, 2, c);
    hold on; grid on;
    
    plot(est_lcss.time, err_lcss.att_trace, 'b-', 'LineWidth', 2, 'DisplayName', 'LCSS-SVD');
    plot(est_mahony_fix.time, err_mahony_fix.att_trace, 'g-.', 'LineWidth', 2, 'DisplayName', 'LCSS-Mahony (Fix)');
    plot(est_mahony_dyn.time, err_mahony_dyn.att_trace, 'r--', 'LineWidth', 2, 'DisplayName', 'LCSS-Mahony (Dyn)');
    
    title(cases(c).label, 'Interpreter','latex', 'FontSize', 20);
    ylabel('$\mathrm{trace}(I_3 - R\hat{R}^{\top})$', 'Interpreter','latex', 'FontSize', 20);
    xlabel('Time (s)', 'Interpreter','latex', 'FontSize', 20);
    
    if c == 1
        legend('show', 'Interpreter','latex', 'Location','best', 'FontSize', 16);
    end

    fprintf('  %s\n', cases(c).label);
    fprintf('    LCSS-SVD         RMSE: %.4f | Final Error: %.4f\n', rmse_lcss.attitude_trace, err_lcss.att_trace(end));
    fprintf('    LCSS-Mahony(Fix) RMSE: %.4f | Final Error: %.4f\n', rmse_mahony_fix.attitude_trace, err_mahony_fix.att_trace(end));
    fprintf('    LCSS-Mahony(Dyn) RMSE: %.4f | Final Error: %.4f\n', rmse_mahony_dyn.attitude_trace, err_mahony_dyn.att_trace(end));
end

%% 6. Format figure
fprintf('[5/5] Saving figure...\n');

if ~isfolder('../figures'), mkdir('../figures'); end
print(gcf, '../figures/CIR_of_LCSS_SVD_and_Mahony_results', '-dpng', '-r300');
fprintf('Figure saved to ../figures/CIR_of_LCSS_SVD_and_Mahony_results.png\n');

%% 7. Print Metrics Table
fprintf('\n===================================================================================================\n');
fprintf('  METRICS COMPARISON TABLE\n');
fprintf('===================================================================================================\n');
fprintf('%-25s | %-12s | %-12s | %-12s | %-12s | %-14s | %-12s\n', ...
    'Observer / Case', 'Overall RMSE', 'SS RMSE', 'Max Error', 'Conv Time(s)', 'Total Time(ms)', 'Step Time(us)');
fprintf('---------------------------------------------------------------------------------------------------\n');

for c = 1:4
    fprintf('%-25s | %-12.4f | %-12.4f | %-12.4f | %-12.4f | %-14.2f | %-12.2f\n', ...
        sprintf('LCSS SVD (C%d)', c), ...
        metrics_lcss.RMSE(c), metrics_lcss.SS_RMSE(c), metrics_lcss.MaxErr(c), ...
        metrics_lcss.ConvTime(c), metrics_lcss.ExecTime(c), metrics_lcss.StepTime(c));
    
    fprintf('%-25s | %-12.4f | %-12.4f | %-12.4f | %-12.4f | %-14.2f | %-12.2f\n', ...
        sprintf('LCSS Mahony(Fix) (C%d)', c), ...
        metrics_mahony_fix.RMSE(c), metrics_mahony_fix.SS_RMSE(c), metrics_mahony_fix.MaxErr(c), ...
        metrics_mahony_fix.ConvTime(c), metrics_mahony_fix.ExecTime(c), metrics_mahony_fix.StepTime(c));

    fprintf('%-25s | %-12.4f | %-12.4f | %-12.4f | %-12.4f | %-14.2f | %-12.2f\n', ...
        sprintf('LCSS Mahony(Dyn) (C%d)', c), ...
        metrics_mahony_dyn.RMSE(c), metrics_mahony_dyn.SS_RMSE(c), metrics_mahony_dyn.MaxErr(c), ...
        metrics_mahony_dyn.ConvTime(c), metrics_mahony_dyn.ExecTime(c), metrics_mahony_dyn.StepTime(c));
    fprintf('---------------------------------------------------------------------------------------------------\n');
end

fprintf('Test complete.\n');
