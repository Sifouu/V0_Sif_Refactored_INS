% compare_lcss_mekf_observers.m
% Tests and compares the LCSS and MEKF attitude observers side-by-side.
% Evaluates 4 measurement cases and plots results in a 2x2 subplot.

clear; clc; close all;
addpath(genpath('../src'));

fprintf('=============================================================\n');
fprintf('  Observer Comparison: LCSS vs MEKF \n');
fprintf('=============================================================\n\n');

%% 1. Ground Truth
fprintf('[1/5] Generating ground truth (1000 Hz, 3 s)...\n');
frequency_gt = 1000;
t_final      = 4;
type         = 'figure8';
perturbation = struct();

[R_true, P_true, V_true, W_true, A_true] = ...
    generate_trajectory(frequency_gt, t_final, type, perturbation);
time_gt = 0:(1/frequency_gt):t_final;

%% 2. Simulate Sensors
fprintf('[2/5] Simulating IMU and magnetometer...\n');

sigma_gyro_val = sqrt(.010);
sigma_acc_val  = sqrt(100); % Increased to account for highly dynamic linear acceleration!
sigma_mag_val  = sqrt(0.0100);

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
    
    % MEKF Parameters
    params_mekf.P0         = 0.01 * eye(3);
    params_mekf.Q_gyro     = params_lcss.Q_gyro;
    params_mekf.g_ref      = g_ref;
    params_mekf.m_ref      = m_ref;
    params_mekf.R_acc      = params_lcss.R_acc;
    params_mekf.acc_axes   = cases(c).acc_axes;
    params_mekf.R_mag      = params_lcss.R_mag;
    params_mekf.mag_axes   = cases(c).mag_axes;
    
    % LCSS Mahony Parameters
    params_mahony = params_lcss;
    params_mahony.k_att = 15; % NCF gain

    % Run Observers
    est_lcss   = lcss_attitude_observer(measurements, init_state, params_lcss);
    est_mekf   = mekf_scalar_attitude_observer(measurements, init_state, params_mekf);
    est_mahony = lcss_mahony_attitude_observer(measurements, init_state, params_mahony);

    % --- Pad estimates with dummy .P and .V ---
    est_lcss.P = zeros(3, N_est); est_lcss.V = zeros(3, N_est);
    est_mekf.P = zeros(3, N_est); est_mekf.V = zeros(3, N_est);
    est_mahony.P = zeros(3, N_est); est_mahony.V = zeros(3, N_est);

    % --- Compute errors ---
    [err_lcss, rmse_lcss] = compute_metrics(gt_ds, est_lcss);
    [err_mekf, rmse_mekf] = compute_metrics(gt_ds, est_mekf);
    [err_mahony, rmse_mahony] = compute_metrics(gt_ds, est_mahony);

    % --- Plotting in 2x2 Grid ---
    subplot(2, 2, c);
    hold on; grid on;
    
    plot(est_lcss.time, err_lcss.att_trace, 'b-', 'LineWidth', 2, 'DisplayName', 'LCSS (Linear SO3 - SVD)');
    plot(est_mahony.time, err_mahony.att_trace, 'g-.', 'LineWidth', 2, 'DisplayName', 'LCSS (Linear SO3 - Mahony)');
    plot(est_mekf.time, err_mekf.att_trace, 'r--', 'LineWidth', 2, 'DisplayName', 'MEKF (Lie-Group)');
    
    title(cases(c).label, 'Interpreter','latex', 'FontSize', 12);
    ylabel('$\mathrm{trace}(I_3 - R\hat{R}^{\top})$', 'Interpreter','latex');
    xlabel('Time (s)', 'Interpreter','latex');
    
    if c == 1
        legend('show', 'Interpreter','latex', 'Location','best');
    end

    fprintf('  %s\n', cases(c).label);
    fprintf('    LCSS-SVD    RMSE: %.4f | Final Error: %.4f\n', rmse_lcss.attitude_trace, err_lcss.att_trace(end));
    fprintf('    LCSS-Mahony RMSE: %.4f | Final Error: %.4f\n', rmse_mahony.attitude_trace, err_mahony.att_trace(end));
    fprintf('    MEKF        RMSE: %.4f | Final Error: %.4f\n', rmse_mekf.attitude_trace, err_mekf.att_trace(end));
end

%% 6. Format figure
fprintf('[5/5] Saving figure...\n');

if ~isfolder('../figures'), mkdir('../figures'); end
print(gcf, '../figures/compare_lcss_mekf_results', '-dpng', '-r300');
fprintf('Figure saved to ../figures/compare_lcss_mekf_results.png\n');
fprintf('Test complete.\n');
