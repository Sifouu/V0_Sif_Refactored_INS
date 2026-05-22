% test_dynamic_mahony.m
% Simulates the thermal shock scenario to compare SVD, Fixed Mahony, 
% and the new Dynamic Mahony (using Trace Ratio of P and M_true).

clear; clc; close all;
addpath(genpath('../src'));

fprintf('=============================================================\n');
fprintf('  Dynamic Mahony Scenario: Nominal Flight (No Bias) \n');
fprintf('=============================================================\n\n');

%% 1. Ground Truth 
fprintf('[1/5] Generating ground truth...\n');
freq_gt = 1000;
t_final = 5; 
[R_true, P_true, V_true, W_true, A_true] = generate_trajectory(freq_gt, t_final, 'figure8', struct());
time_gt = 0:(1/freq_gt):t_final;

%% 2. Simulate Sensors
fprintf('[2/5] Simulating Clean Sensors (No Bias)...\n');
freq_imu = 100;
time_imu = 0:(1/freq_imu):t_final;
N_imu = numel(time_imu);

w_imu = zeros(3, N_imu);
a_imu = zeros(3, N_imu);
mag_meas = zeros(3, N_imu);

sigma_gyro = sqrt(0.001);
sigma_acc  = sqrt(0.1);
sigma_mag  = sqrt(0.01);

for k = 1:N_imu
    t = time_imu(k);
    [~, idx] = min(abs(time_gt - t));
    R = R_true{idx};
    
    W_t = W_true(:, idx);
    w_imu(:, k) = W_t + sigma_gyro * randn(3,1);
    
    g_B = -R' * [0; 0; 9.81];
    a_imu(:, k) = g_B + sigma_acc * randn(3,1);
    
    m_B = R' * [1/sqrt(2); 0; 1/sqrt(2)];
    mag_meas(:, k) = m_B + sigma_mag * randn(3,1);
end

measurements.imu.time = time_imu;
measurements.imu.w_B = w_imu;
measurements.imu.a_B = a_imu;
measurements.mag.time = time_imu;
measurements.mag.mag_meas = mag_meas;

%% 3. Observers Setup
params.P0 = 0.01 * eye(9);
params.Q_gyro = diag(repmat(sigma_gyro^2, 3, 1));
params.R_acc = diag(repmat(sigma_acc^2, 3, 1)); 
params.R_mag = diag(repmat(sigma_mag^2, 3, 1)); 
params.acc_axes = [1;2;3];
params.mag_axes = [1;2;3];
params.k_att = 15; % Fixed Mahony Gain
params.g_ref = [0; 0; 9.81];
params.m_ref = [1/sqrt(2); 0; 1/sqrt(2)];

init_state.R = eye(3);

%% 4. Run Observers
fprintf('[3/5] Running Observers...\n');
est_svd = custom_lcss_svd(measurements, init_state, params);
est_mahony_fixed = custom_lcss_mahony(measurements, init_state, params);

% Dynamic Mahony Tuning (Alpha)
params.dynamic_alpha = 4; 
[est_mahony_dyn, kp_hist] = custom_dynamic_mahony(measurements, init_state, params);

%% 5. Evaluate and Plot
fprintf('[5/5] Computing metrics and plotting...\n');
gt_ds.P = zeros(3, N_imu); gt_ds.V = zeros(3, N_imu);
gt_ds.R = cell(1, N_imu);
for k = 1:N_imu
    [~, idx] = min(abs(time_gt - time_imu(k)));
    gt_ds.R{k} = R_true{idx};
end

est_svd.P = zeros(3, N_imu); est_svd.V = zeros(3, N_imu);
est_mahony_fixed.P = zeros(3, N_imu); est_mahony_fixed.V = zeros(3, N_imu);
est_mahony_dyn.P = zeros(3, N_imu); est_mahony_dyn.V = zeros(3, N_imu);

[err_svd, rmse_svd] = compute_metrics(gt_ds, est_svd);
[err_mahony_fixed, rmse_mahony_fixed] = compute_metrics(gt_ds, est_mahony_fixed);
[err_mahony_dyn, rmse_mahony_dyn] = compute_metrics(gt_ds, est_mahony_dyn);

fprintf('\n=== Trace Error Metrics ===\n');
fprintf('LCSS-SVD         RMSE: %.4f\n', rmse_svd.attitude_trace);
fprintf('LCSS-Mahony(Fix) RMSE: %.4f\n', rmse_mahony_fixed.attitude_trace);
fprintf('LCSS-Mahony(Dyn) RMSE: %.4f\n', rmse_mahony_dyn.attitude_trace);

figure('Position', [100, 100, 1200, 800]);

% Top: Attitude Error
subplot(2,1,1); hold on;
plot(est_svd.time, err_svd.att_trace, 'b-', 'LineWidth', 1.5, 'DisplayName', 'LCSS-SVD');
plot(est_mahony_fixed.time, err_mahony_fixed.att_trace, 'g-', 'LineWidth', 1.5, 'DisplayName', 'LCSS-Mahony (Fixed)');
plot(est_mahony_dyn.time, err_mahony_dyn.att_trace, 'r-', 'LineWidth', 2, 'DisplayName', 'LCSS-Mahony (Dynamic Trace)');
legend('Location', 'best', 'FontSize', 12);
title('Attitude Error Metric: Nominal Flight');
ylabel('Trace Error'); xlabel('Time (s)'); grid on;

% Bottom: Dynamic k_p
subplot(2,1,2); hold on;
yline(15, 'g--', 'LineWidth', 2, 'DisplayName', 'Fixed k_{att} = 15');
plot(time_imu(1:end-1), kp_hist, 'r-', 'LineWidth', 2, 'DisplayName', 'Dynamic k_p(t)');
title('Dynamic Proportional Gain (k_p) over Time');
ylabel('Gain Value'); xlabel('Time (s)'); grid on; legend('Location', 'best');

if ~isfolder('../figures'), mkdir('../figures'); end
print(gcf, '../figures/test_dynamic_mahony', '-dpng', '-r300');
fprintf('Figure saved to ../figures/test_dynamic_mahony.png\n');

%% LOCAL FUNCTIONS: Custom Observers
function estimates = custom_lcss_svd(measurements, init_state, params)
    time_imu = measurements.imu.time; w_imu = measurements.imu.w_B; g_B_imu = measurements.imu.a_B;   
    time_mag = measurements.mag.time; m_B_mag = measurements.mag.mag_meas;  
    N = numel(time_imu); dt = time_imu(2) - time_imu(1);       
    P_cov = params.P0; Q_gyro = params.Q_gyro; R_acc = params.R_acc; R_mag = params.R_mag;
    g_ref = params.g_ref; m_ref = params.m_ref; acc_axes = params.acc_axes; mag_axes = params.mag_axes;
    ka = length(acc_axes); km = length(mag_axes); basis = eye(3);
    C_acc = zeros(ka, 9); for i = 1:ka, C_acc(i, :) = -kron(g_ref, basis(acc_axes(i), :)')'; end
    C_mag = zeros(km, 9); for i = 1:km, C_mag(i, :) = kron(m_ref, basis(mag_axes(i), :)')'; end

    R_init = ALLFUNCS.orthogonalize(init_state.R); x = reshape(R_init', 9, 1);
    estimates.R = cell(1, N); estimates.time = time_imu; estimates.R{1} = R_init; i_mag = 1; 

    for k = 1:N-1
        w_B = w_imu(:, k);
        N_mat = -[ALLFUNCS.skew(x(1:3)); ALLFUNCS.skew(x(4:6)); ALLFUNCS.skew(x(7:9))]; 
        M_k = dt * N_mat * Q_gyro * N_mat'; dR = ALLFUNCS.Rexp(w_B * dt); A_dis = blkdiag(dR', dR', dR');
        P_cov = A_dis * P_cov * A_dis' +  M_k ; P_cov = 0.5 * (P_cov + P_cov');

        Q_acc_k = R_acc(acc_axes, acc_axes); S_acc = C_acc * P_cov * C_acc' + Q_acc_k; K_acc = P_cov * C_acc' / S_acc;
        innov_acc = g_B_imu(acc_axes, k) - C_acc * (A_dis * x); x = A_dis * x + K_acc * innov_acc;
        P_cov = (eye(9) - K_acc * C_acc) * P_cov; P_cov = 0.5 * (P_cov + P_cov');

        if ~isempty(C_mag) && i_mag <= numel(time_mag) && time_imu(k) >= time_mag(i_mag)
            Q_mag_k = R_mag(mag_axes, mag_axes); S_mag = C_mag * P_cov * C_mag' + Q_mag_k; K_mag = P_cov * C_mag' / S_mag;
            innov_mag = m_B_mag(mag_axes, i_mag) - C_mag * x; x = x + K_mag * innov_mag;
            P_cov = (eye(9) - K_mag * C_mag) * P_cov; P_cov = 0.5 * (P_cov + P_cov'); i_mag = i_mag + 1;
        end

        R_bar = reshape(x, 3, 3)'; 
        R_k = ALLFUNCS.orthogonalize(R_bar); x = reshape(R_k', 9, 1);        
        estimates.R{k+1} = R_k;
    end
end

function estimates = custom_lcss_mahony(measurements, init_state, params)
    time_imu = measurements.imu.time; w_imu = measurements.imu.w_B; g_B_imu = measurements.imu.a_B;   
    time_mag = measurements.mag.time; m_B_mag = measurements.mag.mag_meas;  
    N = numel(time_imu); dt = time_imu(2) - time_imu(1);       
    P_cov = params.P0; Q_gyro = params.Q_gyro; R_acc = params.R_acc; R_mag = params.R_mag;
    g_ref = params.g_ref; m_ref = params.m_ref; acc_axes = params.acc_axes; mag_axes = params.mag_axes;
    k_att = params.k_att;
    ka = length(acc_axes); km = length(mag_axes); basis = eye(3);
    C_acc = zeros(ka, 9); for i = 1:ka, C_acc(i, :) = -kron(g_ref, basis(acc_axes(i), :)')'; end
    C_mag = zeros(km, 9); for i = 1:km, C_mag(i, :) = kron(m_ref, basis(mag_axes(i), :)')'; end

    R_init = ALLFUNCS.orthogonalize(init_state.R); x = reshape(R_init', 9, 1);
    estimates.R = cell(1, N); estimates.time = time_imu; estimates.R{1} = R_init;
    current_R = R_init; i_mag = 1;

    for k = 1:N-1
        w_B = w_imu(:, k);
        N_mat = -[ALLFUNCS.skew(x(1:3)); ALLFUNCS.skew(x(4:6)); ALLFUNCS.skew(x(7:9))]; 
        M_k = dt * N_mat * Q_gyro * N_mat'; dR = ALLFUNCS.Rexp(w_B * dt); A_dis = blkdiag(dR', dR', dR');
        P_cov = A_dis * P_cov * A_dis' + M_k; P_cov = 0.5 * (P_cov + P_cov');

        Q_acc_k = R_acc(acc_axes, acc_axes); S_acc = C_acc * P_cov * C_acc' + Q_acc_k; K_acc = P_cov * C_acc' / S_acc;
        innov_acc = g_B_imu(acc_axes, k) - C_acc * (A_dis * x); x = A_dis * x + K_acc * innov_acc;
        P_cov = (eye(9) - K_acc * C_acc) * P_cov; P_cov = 0.5 * (P_cov + P_cov');

        if ~isempty(C_mag) && i_mag <= numel(time_mag) && time_imu(k) >= time_mag(i_mag)
            Q_mag_k = R_mag(mag_axes, mag_axes); S_mag = C_mag * P_cov * C_mag' + Q_mag_k; K_mag = P_cov * C_mag' / S_mag;
            innov_mag = m_B_mag(mag_axes, i_mag) - C_mag * x; x = x + K_mag * innov_mag;
            P_cov = (eye(9) - K_mag * C_mag) * P_cov; P_cov = 0.5 * (P_cov + P_cov'); i_mag = i_mag + 1;
        end

        v1 = x(1:3); v2 = x(4:6); v3 = x(7:9);
        sigma = cross(v1, current_R' * [1; 0; 0]) + cross(v2, current_R' * [0; 1; 0]) + cross(v3, current_R' * [0; 0; 1]);
        w_total = w_B + k_att * sigma;
        current_R = current_R * ALLFUNCS.Rexp(w_total * dt);
        x = reshape(current_R', 9, 1);  
        estimates.R{k+1} = current_R;
    end
end

function [estimates, kp_hist] = custom_dynamic_mahony(measurements, init_state, params)
    time_imu = measurements.imu.time; w_imu = measurements.imu.w_B; g_B_imu = measurements.imu.a_B;   
    time_mag = measurements.mag.time; m_B_mag = measurements.mag.mag_meas;  
    N = numel(time_imu); dt = time_imu(2) - time_imu(1);       
    P_cov = params.P0; Q_gyro = params.Q_gyro; R_acc = params.R_acc; R_mag = params.R_mag;
    g_ref = params.g_ref; m_ref = params.m_ref; acc_axes = params.acc_axes; mag_axes = params.mag_axes;
    alpha = params.dynamic_alpha;
    ka = length(acc_axes); km = length(mag_axes); basis = eye(3);
    C_acc = zeros(ka, 9); for i = 1:ka, C_acc(i, :) = -kron(g_ref, basis(acc_axes(i), :)')'; end
    C_mag = zeros(km, 9); for i = 1:km, C_mag(i, :) = kron(m_ref, basis(mag_axes(i), :)')'; end

    R_init = ALLFUNCS.orthogonalize(init_state.R); x = reshape(R_init', 9, 1);
    estimates.R = cell(1, N); estimates.time = time_imu; estimates.R{1} = R_init;
    current_R = R_init; i_mag = 1; kp_hist = zeros(1, N-1);

    for k = 1:N-1
        w_B = w_imu(:, k);
        N_mat = -[ALLFUNCS.skew(x(1:3)); ALLFUNCS.skew(x(4:6)); ALLFUNCS.skew(x(7:9))]; 
        
        % True process noise for ratio calculation
        M_true = dt * N_mat * Q_gyro * N_mat';
        
        % Inflated process noise for state prediction (to keep Mahony stable)
        M_k = 2000 * M_true; 
        
        dR = ALLFUNCS.Rexp(w_B * dt); A_dis = blkdiag(dR', dR', dR');
        P_cov = A_dis * P_cov * A_dis' + M_k; P_cov = 0.5 * (P_cov + P_cov');

        Q_acc_k = R_acc(acc_axes, acc_axes); S_acc = C_acc * P_cov * C_acc' + Q_acc_k; K_acc = P_cov * C_acc' / S_acc;
        innov_acc = g_B_imu(acc_axes, k) - C_acc * (A_dis * x); x = A_dis * x + K_acc * innov_acc;
        P_cov = (eye(9) - K_acc * C_acc) * P_cov; P_cov = 0.5 * (P_cov + P_cov');

        if ~isempty(C_mag) && i_mag <= numel(time_mag) && time_imu(k) >= time_mag(i_mag)
            Q_mag_k = R_mag(mag_axes, mag_axes); S_mag = C_mag * P_cov * C_mag' + Q_mag_k; K_mag = P_cov * C_mag' / S_mag;
            innov_mag = m_B_mag(mag_axes, i_mag) - C_mag * x; x = x + K_mag * innov_mag;
            P_cov = (eye(9) - K_mag * C_mag) * P_cov; P_cov = 0.5 * (P_cov + P_cov'); i_mag = i_mag + 1;
        end
        
        % Compute dynamic k_p using Trace Ratio
        v1 = x(1:3); v2 = x(4:6); v3 = x(7:9);
        sigma = cross(v1, current_R' * [1; 0; 0]) + cross(v2, current_R' * [0; 1; 0]) + cross(v3, current_R' * [0; 0; 1]);
        
        % Innovation Error Magnitude Tuning
        error_mag = norm(innov_acc)+norm(innov_mag);
        k_min = 10.0; 
        k_p = k_min + alpha * error_mag; % Scales gain proportionally to error
        
        kp_hist(k) = k_p;
        w_total = w_B + k_p * sigma;
        current_R = current_R * ALLFUNCS.Rexp(w_total * dt);
        x = reshape(current_R', 9, 1);  
        estimates.R{k+1} = current_R;
    end
end
