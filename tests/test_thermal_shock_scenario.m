% test_thermal_shock_scenario.m
% Simulates a thermal shock scenario where severe non-linear drift
% affects the gyroscope, while accelerometer and magnetometer remain clean.

clear; clc; close all;
addpath(genpath('../src'));

fprintf('=============================================================\n');
fprintf('  Thermal Shock Scenario: Gyro Bias Ramp \n');
fprintf('=============================================================\n\n');

%% 1. Ground Truth 
fprintf('[1/5] Generating ground truth...\n');
freq_gt = 1000;
t_final = 10; % 10 seconds to show the ramp and its effects
[R_true, P_true, V_true, W_true, A_true] = generate_trajectory(freq_gt, t_final, 'figure8', struct());
time_gt = 0:(1/freq_gt):t_final;

%% 2. Simulate Sensors with Thermal Shock Gyro Bias Ramp
fprintf('[2/5] Simulating Sensors with Thermal Shock Bias Ramp...\n');
freq_imu = 100;
time_imu = 0:(1/freq_imu):t_final;
N_imu = numel(time_imu);

w_imu = zeros(3, N_imu);
a_imu = zeros(3, N_imu);
mag_meas = zeros(3, N_imu);

sigma_gyro = sqrt(0.001);
sigma_acc  = sqrt(0.1);
sigma_mag  = sqrt(0.01);

w_true_history = zeros(3, N_imu);

for k = 1:N_imu
    t = time_imu(k);
    [~, idx] = min(abs(time_gt - t));
    R = R_true{idx};
    
    % Thermal Shock: Bias ramps from 0 to 8 deg/s (0.1396 rad/s) on pitch/yaw (y and z)
    % Ramp starts at 2s and ends at 5s.
    if t < 2
        bias = [0; 0; 0];
    elseif t <= 5
        ramp_factor = (t - 2) / 3; % 0 to 1
        % Non-linear drift (e.g. quadratic)
        bias = [0; 0.1396 * ramp_factor^2; 0.1396 * ramp_factor^2];
    else
        bias = [0; 0.1396; 0.1396];
    end
    
    W_t = W_true(:, idx);
    w_true_history(:, k) = W_t;
    w_imu(:, k) = W_t + bias + sigma_gyro * randn(3,1);
    
    % True gravity in body frame (clean)
    g_B = -R' * [0; 0; 9.81];
    a_imu(:, k) = g_B + sigma_acc * randn(3,1);
    
    % Magnetometer (clean)
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
params.k_att = 15; % Mahony NCF gain
params.g_ref = [0; 0; 9.81];
params.m_ref = [1/sqrt(2); 0; 1/sqrt(2)];

init_state.R = eye(3);

%% 4. Run Observers
fprintf('[3/5] Running Custom LCSS-SVD...\n');
[est_svd, det_R_svd] = custom_lcss_svd(measurements, init_state, params);

fprintf('[4/5] Running Custom LCSS-Mahony...\n');
[est_mahony, det_R_mahony] = custom_lcss_mahony(measurements, init_state, params);

%% 5. Evaluate and Plot
fprintf('[5/5] Computing metrics and plotting...\n');
gt_ds.P = zeros(3, N_imu); gt_ds.V = zeros(3, N_imu);
gt_ds.R = cell(1, N_imu);
for k = 1:N_imu
    [~, idx] = min(abs(time_gt - time_imu(k)));
    gt_ds.R{k} = R_true{idx};
end

est_svd.P = zeros(3, N_imu); est_svd.V = zeros(3, N_imu);
est_mahony.P = zeros(3, N_imu); est_mahony.V = zeros(3, N_imu);

[err_svd, rmse_svd] = compute_metrics(gt_ds, est_svd);
[err_mahony, rmse_mahony] = compute_metrics(gt_ds, est_mahony);

fprintf('\n=== Trace Error Metrics ===\n');
fprintf('LCSS-SVD    RMSE: %f | Final: %f\n', rmse_svd.attitude_trace, err_svd.att_trace(end));
fprintf('LCSS-Mahony RMSE: %f | Final: %f\n', rmse_mahony.attitude_trace, err_mahony.att_trace(end));

figure('Position', [100, 100, 1200, 1000]);

% Top: Gyroscope Profile
subplot(3,1,1);
hold on;
plot(time_imu, w_imu(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Measured Pitch Rate (with bias)');
plot(time_imu, w_true_history(2,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'True Pitch Rate');
plot(time_imu, w_imu(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Measured Yaw Rate (with bias)');
plot(time_imu, w_true_history(3,:), 'g--', 'LineWidth', 1.5, 'DisplayName', 'True Yaw Rate');
title('Gyroscope Profile (Thermal Shock Non-linear Bias Ramp)');
ylabel('rad/s'); grid on; legend('Location', 'best');

% Mid: Determinant
subplot(3,1,2);
plot(time_imu(1:end-1), det_R_svd, 'b-', 'LineWidth', 4, 'DisplayName', 'det(R_bar) for LCSS-SVD');
hold on;
plot(time_imu(1:end-1), det_R_mahony, 'y--', 'LineWidth', 2, 'DisplayName', 'det(R_bar) for LCSS-Mahony');
title('Determinant of the Unconstrained Linear State (R_bar)');
ylabel('det(R)'); xlabel('Time (s)'); grid on; legend('Location', 'best');
ylim([0.8 1.2]);

% Bottom: Attitude Error
subplot(3,1,3);
hold on;
plot(est_svd.time, err_svd.att_trace, 'b-', 'LineWidth', 1.5, 'DisplayName', 'LCSS-SVD');
plot(est_mahony.time, err_mahony.att_trace, 'g-', 'LineWidth', 2, 'DisplayName', 'LCSS-Mahony');
legend('Location', 'best', 'FontSize', 14);
title('Attitude Error Metric: Response to Thermal Shock Drift');
ylabel('Trace Error'); xlabel('Time (s)');
grid on;

if ~isfolder('../figures'), mkdir('../figures'); end
print(gcf, '../figures/test_thermal_shock_scenario', '-dpng', '-r300');
fprintf('Figure saved to ../figures/test_thermal_shock_scenario.png\n');


%% LOCAL FUNCTIONS: Custom Observers with Det Logging
function [estimates, det_R] = custom_lcss_svd(measurements, init_state, params)
    time_imu = measurements.imu.time; w_imu = measurements.imu.w_B; g_B_imu = measurements.imu.a_B;   
    time_mag = measurements.mag.time; m_B_mag = measurements.mag.mag_meas;  
    N = numel(time_imu); dt = time_imu(2) - time_imu(1);       
    P_cov = params.P0; Q_gyro = params.Q_gyro; R_acc = params.R_acc; R_mag = params.R_mag;
    g_ref = params.g_ref; m_ref = params.m_ref; acc_axes = params.acc_axes; mag_axes = params.mag_axes;
    ka = length(acc_axes); km = length(mag_axes); basis = eye(3);
    C_acc = zeros(ka, 9); for i = 1:ka, C_acc(i, :) = -kron(g_ref, basis(acc_axes(i), :)')'; end
    C_mag = zeros(km, 9); for i = 1:km, C_mag(i, :) = kron(m_ref, basis(mag_axes(i), :)')'; end

    R_init = ALLFUNCS.orthogonalize(init_state.R); x = reshape(R_init', 9, 1);
    estimates.R = cell(1, N); estimates.time = time_imu; estimates.R{1} = R_init;
    i_mag = 1; det_R = zeros(1, N-1);

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
        det_R(k) = det(R_bar);
        R_k = ALLFUNCS.orthogonalize(R_bar);
        x = reshape(R_k', 9, 1);        
        estimates.R{k+1} = R_k;
    end
end

function [estimates, det_R] = custom_lcss_mahony(measurements, init_state, params)
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
    current_R = R_init; i_mag = 1; det_R = zeros(1, N-1);

    for k = 1:N-1
        w_B = w_imu(:, k);
        N_mat = -[ALLFUNCS.skew(x(1:3)); ALLFUNCS.skew(x(4:6)); ALLFUNCS.skew(x(7:9))]; 
        M_k = 2000*dt * N_mat * Q_gyro * N_mat'; dR = ALLFUNCS.Rexp(w_B * dt); A_dis = blkdiag(dR', dR', dR');
        P_cov = A_dis * P_cov * A_dis' + M_k; P_cov = 0.5 * (P_cov + P_cov');

        Q_acc_k = R_acc(acc_axes, acc_axes); S_acc = C_acc * P_cov * C_acc' + Q_acc_k; K_acc = P_cov * C_acc' / S_acc;
        innov_acc = g_B_imu(acc_axes, k) - C_acc * (A_dis * x); x = A_dis * x + K_acc * innov_acc;
        P_cov = (eye(9) - K_acc * C_acc) * P_cov; P_cov = 0.5 * (P_cov + P_cov');

        if ~isempty(C_mag) && i_mag <= numel(time_mag) && time_imu(k) >= time_mag(i_mag)
            Q_mag_k = R_mag(mag_axes, mag_axes); S_mag = C_mag * P_cov * C_mag' + Q_mag_k; K_mag = P_cov * C_mag' / S_mag;
            innov_mag = m_B_mag(mag_axes, i_mag) - C_mag * x; x = x + K_mag * innov_mag;
            P_cov = (eye(9) - K_mag * C_mag) * P_cov; P_cov = 0.5 * (P_cov + P_cov'); i_mag = i_mag + 1;
        end

        R_bar = reshape(x, 3, 3)'; 
        det_R(k) = det(R_bar);

        v1 = x(1:3); v2 = x(4:6); v3 = x(7:9);
        sigma = cross(v1, current_R' * [1; 0; 0]) + cross(v2, current_R' * [0; 1; 0]) + cross(v3, current_R' * [0; 0; 1]);
        w_total = w_B + k_att * sigma;
        current_R = current_R * ALLFUNCS.Rexp(w_total * dt);
        x = reshape(current_R', 9, 1);  
        estimates.R{k+1} = current_R;
    end
end
