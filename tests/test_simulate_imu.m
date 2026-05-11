% test_simulate_imu.m
% Script to verify the IMU simulation function (downsampling, noise, biases)

clear; clc; close all;
addpath(genpath('../src'));

%% 1. Generate High-Frequency Ground Truth
frequency_gt = 1000; % 1000 Hz Ground truth
t_final = 15;
type = 'figure8';
perturbation = struct();

disp('Generating ground truth (1000 Hz)...');
[R_true, P_true, V_true, W_true, A_true] = generate_trajectory(frequency_gt, t_final, type, perturbation);
time_gt = 0:(1/frequency_gt):t_final;

%% 2. Setup IMU Parameters
% We will downsample to 200 Hz to prove the interpolation works
imu_params.frequency = 200; 
imu_params.accel_noise_std = 0.05;  % Noticeable noise
imu_params.gyro_noise_std = 0.01;
imu_params.accel_bias = [0.1; -0.2; 0.05]; % Constant bias
imu_params.gyro_bias = [0.02; -0.01; 0.03];
imu_params.g = 9.81;

%% 3. Simulate IMU
disp('Simulating IMU measurements (200 Hz)...');
tic;
[a_B_meas, w_B_meas, time_imu] = simulate_imu(A_true, W_true, R_true, time_gt, imu_params);
toc;

%% 4. Plot Results
% Gyroscope Plot
figure('Name', 'Gyroscope Measurements', 'Color', 'w');
for i = 1:3
    subplot(3,1,i);
    % Plot True vs Measured
    plot(time_gt, W_true(i,:), 'k-', 'LineWidth', 1.5); hold on;
    plot(time_imu, w_B_meas(i,:), 'r.', 'MarkerSize', 4);
    
    % Draw a line showing the bias
    plot(time_gt, W_true(i,:) + imu_params.gyro_bias(i), 'b--', 'LineWidth', 1);
    
    ylabel(sprintf('W_%d [rad/s]', i));
    if i == 1
        title('Gyroscope: True vs Measured (with Bias & Noise)'); 
        legend('True', 'Measured', 'True + Bias', 'Location', 'best'); 
    end
    grid on;
end
xlabel('Time [s]');

% Accelerometer Plot
% To plot truth, we need to compute true specific force
M = length(time_gt);
a_B_true_sf = zeros(3, M);
for k = 1:M
    a_B_true_sf(:, k) = R_true{k}' * (A_true(:, k) - [0; 0; imu_params.g]);
end

figure('Name', 'Accelerometer Measurements', 'Color', 'w');
for i = 1:3
    subplot(3,1,i);
    plot(time_gt, a_B_true_sf(i,:), 'k-', 'LineWidth', 1.5); hold on;
    plot(time_imu, a_B_meas(i,:), 'g.', 'MarkerSize', 4);
    
    % Draw a line showing the bias
    plot(time_gt, a_B_true_sf(i,:) + imu_params.accel_bias(i), 'b--', 'LineWidth', 1);
    
    ylabel(sprintf('a_%d [m/s^2]', i));
    if i == 1
        title('Accelerometer (Specific Force): True vs Measured'); 
        legend('True Spec Force', 'Measured', 'True + Bias', 'Location', 'best'); 
    end
    grid on;
end
xlabel('Time [s]');

disp('IMU verification complete.');
