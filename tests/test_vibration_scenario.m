% test_vibration_scenario.m
% Implements the "Unbalanced Micro-Quad" scenario to break the SVD observer

clear; clc; close all;
addpath(genpath('../src'));

fprintf('=============================================================\n');
fprintf('  Vibration Scenario: Breaking the SVD Observer\n');
fprintf('=============================================================\n\n');

%% 1. Ground Truth (High frequency to allow true high-freq vibration)
fprintf('[1/5] Generating ground truth...\n');
freq_gt = 1000;
t_final = 5; 
[R_true, P_true, V_true, W_true, A_true] = generate_trajectory(freq_gt, t_final, 'figure8', struct());
time_gt = 0:(1/freq_gt):t_final;

%% 2. Sensor Simulation with Extreme Disturbances
fprintf('[2/5] Simulating Aliased Vibration & Magnetic Impulses...\n');
% IMU at 100 Hz to cause aliasing of a 150 Hz vibration
freq_imu = 100;
time_imu = 0:(1/freq_imu):t_final;
N_imu = numel(time_imu);

w_imu = zeros(3, N_imu);
a_imu = zeros(3, N_imu); 
mag_meas = zeros(3, N_imu);

% Vibration params (150 Hz sampled at 100 Hz aliases to 50 Hz)
f_vib = 142; % Hz
A_vib = 30;  % m/s^2 (approx 3g amplitude)

% Mag impulse params
mag_impulse_prob = 0.02; % 2% chance per step
mag_impulse_mag  = 2.0;

sigma_gyro = 0.01;
sigma_acc  = 0.1;
sigma_mag  = 0.01;

for k = 1:N_imu
    t = time_imu(k);
    [~, idx] = min(abs(time_gt - t));
    R = R_true{idx};
    
    w_imu(:, k) = W_true(:, idx) + sigma_gyro * randn(3,1);
    
    % True gravity in body frame
    g_B = -R' * [0; 0; 9.81];
    
    % Add high-freq deterministic vibration
    vib = A_vib * sin(2*pi*f_vib*t) * [1; 1; 1] / sqrt(3); 
    
    a_imu(:, k) = g_B + vib + sigma_acc * randn(3,1);
    
    % Magnetometer
    m_B = R' * [1/sqrt(2); 0; 1/sqrt(2)];
    if rand() < mag_impulse_prob
        m_B = m_B + mag_impulse_mag * randn(3,1); % impulse
    else
        m_B = m_B + sigma_mag * randn(3,1);
    end
    mag_meas(:, k) = m_B;
end

measurements.imu.time = time_imu;
measurements.imu.w_B = w_imu;
measurements.imu.a_B = a_imu;
measurements.mag.time = time_imu;
measurements.mag.mag_meas = mag_meas;

%% 3. Observers Setup
params.P0 = 0.01 * eye(9);
params.Q_gyro = diag(repmat(sigma_gyro^2, 3, 1));
params.R_acc = diag(repmat((sigma_acc*10)^2, 3, 1)); % Inflate noise due to vibration
params.R_mag = diag(repmat((sigma_mag*10)^2, 3, 1)); 
params.acc_axes = [1;2;3];
params.mag_axes = [1;2;3];
params.k_att = 15; % Mahony NCF gain
params.g_ref = [0; 0; 9.81];
params.m_ref = [1/sqrt(2); 0; 1/sqrt(2)];

init_state.R = eye(3);

%% 4. Run Observers
fprintf('[3/5] Running standard LCSS-Mahony observer...\n');
est_mahony = lcss_mahony_attitude_observer(measurements, init_state, params);

fprintf('[4/5] Running custom logging LCSS-SVD observer...\n');
[est_svd, det_R_bar, S_bar] = custom_lcss_svd(measurements, init_state, params);

%% 5. Evaluate and Plot
fprintf('[5/5] Computing metrics and plotting...\n');
gt_ds.P = zeros(3, N_imu); gt_ds.V = zeros(3, N_imu);
gt_ds.R = cell(1, N_imu);
for k = 1:N_imu
    [~, idx] = min(abs(time_gt - time_imu(k)));
    gt_ds.R{k} = R_true{idx};
end

% Pad P and V so compute_metrics doesn't crash
est_svd.P = zeros(3, N_imu); est_svd.V = zeros(3, N_imu);
est_mahony.P = zeros(3, N_imu); est_mahony.V = zeros(3, N_imu);

[err_svd, rmse_svd] = compute_metrics(gt_ds, est_svd);
[err_mahony, rmse_mahony] = compute_metrics(gt_ds, est_mahony);

figure('Position', [100, 100, 1200, 1000]);

% Top: Accelerometer with Vibration
subplot(3,1,1);
plot(time_imu, a_imu(1,:), 'r', time_imu, a_imu(2,:), 'g', time_imu, a_imu(3,:), 'b');
title('Accelerometer Measurements (Aliased 150Hz Vibration at 100Hz)');
ylabel('m/s^2'); grid on;

% Mid: Determinant and S_min
subplot(3,1,2);
yyaxis left
plot(time_imu(1:end-1), det_R_bar, 'b-', 'LineWidth', 1.5);
ylabel('det(R_bar)');
ylim([-1 1.5]);
yline(0, 'k--', 'LineWidth', 2);
yyaxis right
plot(time_imu(1:end-1), S_bar(3,:), 'r-', 'LineWidth', 1.5);
ylabel('Min Singular Value');
title('Mathematical Failure: Determinant Zero-Crossing and Eigenvalue Compression');
grid on;

% Bottom: Attitude Error
subplot(3,1,3);
hold on;
plot(est_svd.time, err_svd.att_trace, 'b-', 'LineWidth', 1.5, 'DisplayName', 'LCSS-SVD');
plot(est_mahony.time, err_mahony.att_trace, 'g-', 'LineWidth', 2, 'DisplayName', 'LCSS-Mahony');
legend('Location', 'best', 'FontSize', 14);
title('Attitude Error: Catastrophic Chattering vs Smooth Tracking');
ylabel('Trace Error'); xlabel('Time (s)');
ylim([0 3]); grid on;

if ~isfolder('../figures'), mkdir('../figures'); end
print(gcf, '../figures/test_vibration_scenario', '-dpng', '-r300');
fprintf('Figure saved to ../figures/test_vibration_scenario.png\n');

%% LOCAL FUNCTION: Custom LCSS SVD logger
function [estimates, det_R, S_vals] = custom_lcss_svd(measurements, init_state, params)
    time_imu = measurements.imu.time;
    w_imu    = measurements.imu.w_B;   
    g_B_imu  = measurements.imu.a_B;   
    time_mag = measurements.mag.time;
    m_B_mag  = measurements.mag.mag_meas;  

    N  = numel(time_imu);
    dt = time_imu(2) - time_imu(1);       
    P_cov = params.P0;
    Q_gyro = params.Q_gyro;
    R_acc  = params.R_acc;
    R_mag  = params.R_mag;
    g_ref = params.g_ref;
    m_ref = params.m_ref;
    acc_axes = params.acc_axes;
    mag_axes = params.mag_axes;

    ka = length(acc_axes);
    km = length(mag_axes);
    basis = eye(3);
    C_acc = zeros(ka, 9);
    for i = 1:ka, C_acc(i, :) = -kron(g_ref, basis(acc_axes(i), :)')'; end
    C_mag = zeros(km, 9);
    for i = 1:km, C_mag(i, :) = kron(m_ref, basis(mag_axes(i), :)')'; end

    R_init = ALLFUNCS.orthogonalize(init_state.R);
    x      = reshape(R_init', 9, 1);
    estimates.R = cell(1, N);
    estimates.time = time_imu;
    estimates.R{1} = R_init;

    i_mag = 1;   
    det_R = zeros(1, N-1);
    S_vals = zeros(3, N-1);

    for k = 1:N-1
        w_B = w_imu(:, k);
        N_mat = -[ALLFUNCS.skew(x(1:3)); ALLFUNCS.skew(x(4:6)); ALLFUNCS.skew(x(7:9))]; 
        M_k   = dt * N_mat * Q_gyro * N_mat';
        dR    = ALLFUNCS.Rexp(w_B * dt);
        A_dis = blkdiag(dR', dR', dR');
        P_cov = A_dis * P_cov * A_dis' +  M_k ;
        P_cov = 0.5 * (P_cov + P_cov');

        Q_acc_k = R_acc(acc_axes, acc_axes);
        S_acc = C_acc * P_cov * C_acc' + Q_acc_k;
        K_acc = P_cov * C_acc' / S_acc;
        y_acc     = g_B_imu(acc_axes, k);          
        innov_acc = y_acc - C_acc * (A_dis * x);   
        x         = A_dis * x + K_acc * innov_acc;
        P_cov = (eye(9) - K_acc * C_acc) * P_cov;
        P_cov = 0.5 * (P_cov + P_cov');

        if ~isempty(C_mag) && i_mag <= numel(time_mag) && time_imu(k) >= time_mag(i_mag)
            Q_mag_k  = R_mag(mag_axes, mag_axes);
            S_mag = C_mag * P_cov * C_mag' + Q_mag_k;
            K_mag = P_cov * C_mag' / S_mag;
            y_mag     = m_B_mag(mag_axes, i_mag);   
            innov_mag = y_mag - C_mag * x;
            x         = x + K_mag * innov_mag;
            P_cov = (eye(9) - K_mag * C_mag) * P_cov;
            P_cov = 0.5 * (P_cov + P_cov');
            i_mag = i_mag + 1;
        end

        % LOG UNCONSTRAINED STATE
        R_bar = reshape(x, 3, 3)'; 
        det_R(k) = det(R_bar);
        [~, S_mat, ~] = svd(R_bar);
        S_vals(:, k) = diag(S_mat);

        % PROJECT
        R_k = ALLFUNCS.orthogonalize(R_bar);
        x   = reshape(R_k', 9, 1);        
        estimates.R{k+1} = R_k;
    end
end
