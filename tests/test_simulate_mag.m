% test_simulate_mag.m
% Script to verify the Magnetometer simulation function (downsampling, noise)

clear; clc; close all;
addpath(genpath('../src'));

%% 1. Generate High-Frequency Ground Truth
frequency_gt = 1000; % 1000 Hz Ground truth
t_final = 15;
type = 'figure8';
perturbation = struct();

disp('Generating ground truth (1000 Hz)...');
% We only need the rotation matrices for the magnetometer
[R_true, ~, ~, ~, ~] = generate_trajectory(frequency_gt, t_final, type, perturbation);
time_gt = 0:(1/frequency_gt):t_final;

%% 2. Setup Magnetometer Parameters
% We will downsample to 50 Hz to prove the downsampling works
mag_params.frequency = 50; 
mag_params.noise_std = 0.05;  % Noticeable noise
mag_params.m_I = [1/sqrt(2); 0; 1/sqrt(2)];

%% 3. Simulate Magnetometer
disp('Simulating Magnetometer measurements (50 Hz)...');
tic;
[m_B_meas, time_mag] = simulate_mag(R_true, time_gt, mag_params);
toc;

%% 4. Plot Results
% To plot truth, we need to compute true magnetic field in body frame
M_gt = length(time_gt);
m_B_true = zeros(3, M_gt);
for k = 1:M_gt
    m_B_true(:, k) = R_true{k}' * mag_params.m_I;
end

figure('Name', 'Magnetometer Measurements', 'Color', 'w');
for i = 1:3
    subplot(3,1,i);
    plot(time_gt, m_B_true(i,:), 'k-', 'LineWidth', 1.5); hold on;
    plot(time_mag, m_B_meas(i,:), 'b.', 'MarkerSize', 6);
    
    ylabel(sprintf('m_{B,%d}', i));
    if i == 1
        title('Magnetometer: True vs Measured (with Noise, No Bias)'); 
        legend('True Magnetic Field', 'Measured (50 Hz)', 'Location', 'best'); 
    end
    grid on;
end
xlabel('Time [s]');

disp('Magnetometer verification complete.');
