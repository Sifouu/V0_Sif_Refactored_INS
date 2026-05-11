% ==========================================================================
%  MAIN_TEMPLATE.m  —  INS Observer Benchmark Template
% ==========================================================================
%
%  PURPOSE:
%    This is the ENTRY POINT script for testing and benchmarking a new
%    INS observer. It is intentionally left incomplete.
%
%    YOUR TASK as the observer designer is to:
%      [1] Tune the sensor and observer parameters (Section 3)
%      [2] Call your own observer function     (Section 4)
%      [3] (Optional) Add your observer to the comparison block (Section 6)
%
%  HOW TO USE:
%    1. Copy this file and rename it, e.g. "main_my_observer.m"
%    2. Implement your observer in src/observers/my_observer.m
%       (start from the template: src/observers/observer_template.m)
%    3. Fill in the TODO sections below
%    4. Run from the root folder: run('main_my_observer.m')
%
%  OUTPUTS:
%    - RMSE metrics printed in the console
%    - Publication-quality 2x2 comparison figure
%    - (Optional) 3D animated trajectory saved to figures/
%
%  ARCHITECTURE REMINDER:
%    generate_trajectory  -->  simulate_imu / simulate_gps /
%                              simulate_landmarks / simulate_mag
%                         -->  measurements struct
%                         -->  [YOUR OBSERVER]
%                         -->  compute_metrics  -->  plot_paper_results
%
% ==========================================================================

clear; clc; close all;
addpath(genpath('src'));   % Load all source modules

fprintf('=========================================================\n');
fprintf('  INS Observer Benchmark — MAIN_TEMPLATE.m\n');
fprintf('=========================================================\n\n');

%% -----------------------------------------------------------------------
%  1. GROUND TRUTH TRAJECTORY
%     Choose the trajectory type and simulation duration.
%     Available types: 'figure8', 'helix', 'random_walk'
% ------------------------------------------------------------------------
frequency_gt = 1000;   % [Hz] Ground truth frequency — do NOT change this
t_final      = 25;     % [s]  Total simulation duration
type         = 'figure8';  % Trajectory type

fprintf('[1/5] Generating ground truth (%d Hz, %ds, type: %s)...\n', ...
        frequency_gt, t_final, type);

perturbation = struct();  % Leave empty for no perturbation
[R_true, P_true, V_true, W_true, A_true] = ...
    generate_trajectory(frequency_gt, t_final, type, perturbation);
time_gt = 0 : (1/frequency_gt) : t_final;

ground_truth.P    = P_true;
ground_truth.V    = V_true;
ground_truth.R    = R_true;
ground_truth.time = time_gt;

%% -----------------------------------------------------------------------
%  2. SENSOR SIMULATION
%     Simulates all sensors at their respective sampling frequencies.
%     Measurements are stored in the 'measurements' struct.
%     This section is ready to use — no modification needed here.
% ------------------------------------------------------------------------
fprintf('[2/5] Simulating sensors...\n');

% --- IMU (high-rate, e.g. 1000 Hz) ---
imu_params.frequency       = 1000;
imu_params.accel_noise_std = 0.05;
imu_params.gyro_noise_std  = 0.01;
imu_params.accel_bias      = zeros(3,1);
imu_params.gyro_bias       = zeros(3,1);
imu_params.g               = 9.81;
[measurements.imu.a_B, measurements.imu.w_B, measurements.imu.time] = ...
    simulate_imu(A_true, W_true, R_true, time_gt, imu_params);

% --- GPS (low-rate, with optional outage) ---
gps_params.frequency      = 5;
gps_params.noise_std_pos  = 0.5;
gps_params.noise_std_vel  = 0.1;
% gps_params.outage_start = 10.0;  % Uncomment to enable GPS outage
% gps_params.outage_end   = 13.0;
[measurements.gps.p_meas, measurements.gps.v_meas, ...
 measurements.gps.is_valid, measurements.gps.time] = ...
    simulate_gps(P_true, V_true, time_gt, gps_params);

% --- Camera / Landmarks (medium-rate) ---
cam_params.frequency         = 20;
cam_params.num_landmarks     = 8;
cam_params.noise_std_pos     = 0.1;
cam_params.noise_std_bearing = 0.01;
[measurements.cam.landmark_meas, measurements.cam.bearing, ...
 measurements.cam.positions_I, measurements.cam.time] = ...
    simulate_landmarks(P_true, R_true, time_gt, cam_params);

% --- Magnetometer (medium-rate) ---
mag_params.frequency = 50;
mag_params.noise_std = 0.02;
[measurements.mag.mag_meas, measurements.mag.time] = ...
    simulate_mag(R_true, time_gt, mag_params);

%% -----------------------------------------------------------------------
%  3. INITIAL STATE & OBSERVER PARAMETERS
%     >>> TODO: Tune these parameters for YOUR observer <<<
%
%     - init_state is deliberately initialised with some errors to
%       make convergence non-trivial (this is realistic).
%     - Add any observer-specific parameters to the 'params' struct
%       (e.g. params.k_att, params.P0, custom gains, etc.)
% ------------------------------------------------------------------------
fprintf('[3/5] Setting up parameters...\n');

% --- Gravity vector (do NOT change) ---
params.g     = 9.81;
params.g_vec = [0; 0; -params.g];

% --- Noise covariance matrices ---
% >>> TODO: Tune these to match your observer's noise model <<<
params.Q_gps = gps_params.noise_std_pos^2 * eye(3);
params.Q_cam = cam_params.noise_std_pos^2  * eye(3);

% --- Initial state with deliberate errors (realistic scenario) ---
init_state.P = P_true(:, 1) + [1.5; -1.5; 0.5];  % Position error [m]
init_state.V = V_true(:, 1) + [0.5; -0.5; 0.0];  % Velocity error [m/s]
theta_err    = 0.1; % [rad] initial attitude error
R_err        = [cos(theta_err) -sin(theta_err) 0;
                sin(theta_err)  cos(theta_err) 0;
                0               0              1];
init_state.R = R_true{1} * R_err;

% >>> TODO: Add your own tuning parameters below <<<
% Example:
%   params.P0    = blkdiag(eye(3), eye(3), 0.1*eye(3));  % Initial covariance
%   params.k_att = 10;                                    % LPV attitude gain
%   params.alpha = 0.5;                                   % Custom gain

%% -----------------------------------------------------------------------
%  4. RUN YOUR OBSERVER
%     >>> TODO: Replace this block with your observer function call <<<
%
%     Your observer MUST return a struct 'estimates' with the fields:
%       estimates.P    — 3xN matrix of estimated positions
%       estimates.V    — 3xN matrix of estimated velocities
%       estimates.R    — 1xN cell array of 3x3 rotation matrices
%       estimates.time — 1xN time vector (should match imu.time)
%
%     See src/observers/observer_template.m for the full contract.
% ------------------------------------------------------------------------
fprintf('[4/5] Running observer...\n');

% =========================================================
% >>> REPLACE THE LINE BELOW WITH YOUR OBSERVER CALL <<<
%
%   estimates = my_observer(measurements, init_state, params);
%
% =========================================================
warning('TEMPLATE: No observer has been called yet. Add your observer call in Section 4.');
estimates = [];  % Remove this line once you add your observer call
% =========================================================

if isempty(estimates)
    fprintf('\n  [!] No observer output detected. Exiting early.\n');
    fprintf('      --> Open MAIN_TEMPLATE.m and complete Section 4.\n\n');
    return;
end

%% -----------------------------------------------------------------------
%  5. EVALUATE RESULTS
%     Computes RMSE metrics and generates plots automatically.
%     No modification needed here.
% ------------------------------------------------------------------------
fprintf('[5/5] Computing metrics and plotting...\n');

% Downsample ground truth to match observer output frequency
N_est = length(estimates.time);
gt_ds.time = estimates.time;
gt_ds.P    = zeros(3, N_est);
gt_ds.V    = zeros(3, N_est);
gt_ds.R    = cell(1, N_est);
for k = 1:N_est
    [~, idx]     = min(abs(time_gt - estimates.time(k)));
    gt_ds.P(:,k) = P_true(:, idx);
    gt_ds.V(:,k) = V_true(:, idx);
    gt_ds.R{k}   = R_true{idx};
end

[errors, rmse] = compute_metrics(gt_ds, estimates);

fprintf('\n=========================================================\n');
fprintf('  RMSE Results\n');
fprintf('=========================================================\n');
fprintf('  Position  (m)   : %.4f\n', rmse.position_euclidean);
fprintf('  Velocity  (m/s) : %.4f\n', rmse.velocity_euclidean);
fprintf('  Attitude (trace): %.6f\n', rmse.attitude_trace);
fprintf('=========================================================\n\n');

% >>> TODO: Replace 'My Observer' with your method name for the legend <<<
plot_paper_results(ground_truth, estimates, errors, 'My Observer');

% --- Optional: Generate animated 3D video ---
% animate_results(ground_truth, estimates, 'My Observer');

fprintf('Done. Figures saved to figures/\n');

%% -----------------------------------------------------------------------
%  6. (OPTIONAL) COMPARE AGAINST BASELINES
%     Uncomment and extend this block to overlay your observer against
%     the provided Generic Observer and EKF baselines.
% ------------------------------------------------------------------------
% est_generic = generic_observer(measurements, init_state, params);
% est_ekf     = ekf_observer(measurements, init_state, params);
%
% [err_generic, rmse_generic] = compute_metrics(gt_ds, est_generic);
% [err_ekf,     rmse_ekf    ] = compute_metrics(gt_ds, est_ekf);
% [err_mine,    rmse_mine   ] = compute_metrics(gt_ds, estimates);
%
% estimates_list = {est_generic, est_ekf, estimates};
% errors_list    = {err_generic, err_ekf, err_mine};
% legend_names   = {'Generic Observer', 'EKF', 'My Observer'};  % <-- change last entry
%
% plot_paper_results(ground_truth, estimates_list, errors_list, legend_names);
