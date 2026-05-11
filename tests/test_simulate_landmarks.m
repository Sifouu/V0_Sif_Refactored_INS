% test_simulate_landmarks.m
% Script to verify the Landmark simulation function

clear; clc; close all;
addpath(genpath('../src'));

%% 1. Generate High-Frequency Ground Truth
frequency_gt = 1000; % 1000 Hz Ground truth
t_final = 15;
type = 'figure8';
perturbation = struct();

disp('Generating ground truth (1000 Hz)...');
[R_true, P_true, ~, ~, ~] = generate_trajectory(frequency_gt, t_final, type, perturbation);
time_gt = 0:(1/frequency_gt):t_final;

%% 2. Setup Landmark Parameters
% Downsample to 20 Hz (typical camera framerate)
landmark_params.frequency = 20; 
landmark_params.num_landmarks = 15; % Request 15 landmarks scattered around
landmark_params.noise_std_pos = 0.05; % 5cm noise for depth measurement
landmark_params.noise_std_bearing = 0.01; % Noise for normalized bearing

%% 3. Simulate Landmarks
disp('Simulating Camera/Landmark measurements (20 Hz)...');
tic;
[y_meas, bearing, positions_I, time_cam] = simulate_landmarks(P_true, R_true, time_gt, landmark_params);
toc;

%% 4. Plot Results

% Figure 1: 3D Environment Map
figure('Name', '3D Environment Map', 'Color', 'w');
plot3(P_true(1,:), P_true(2,:), P_true(3,:), 'k-', 'LineWidth', 1.5); hold on;
plot3(positions_I(1,:), positions_I(2,:), positions_I(3,:), 'r^', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Drone Trajectory and Automatically Generated Landmarks');
legend('Drone Trajectory', 'Landmarks (Randomly Placed)', 'Location', 'best');
view(3);

% Figure 2: Verification of Bearing Vector (Norm should be 1)
figure('Name', 'Bearing Vector Norm Check', 'Color', 'w');
bearing_norms = squeeze(sqrt(sum(bearing(:,:,1).^2, 1))); % Norms of Landmark 1 over time
plot(time_cam, bearing_norms, 'b.', 'MarkerSize', 8);
ylim([0.9 1.1]);
ylabel('|| bearing ||'); xlabel('Time [s]');
title('Norm of Measured Bearing Vectors for Landmark 1 (Should be strictly 1)');
grid on;

% Figure 3: Relative Position Measurements (Landmark 1)
figure('Name', 'Relative Position (Landmark 1)', 'Color', 'w');
M_cam = length(time_cam);
y_true_L1 = zeros(3, M_cam);
P_interp = interp1(time_gt, P_true', time_cam, 'linear')';

% Recompute truth for plotting
for k = 1:M_cam
    [~, idx] = min(abs(time_gt - time_cam(k)));
    R = R_true{idx};
    y_true_L1(:, k) = R' * (positions_I(:, 1) - P_interp(:, k));
end

for i = 1:3
    subplot(3,1,i);
    plot(time_cam, y_true_L1(i,:), 'k-', 'LineWidth', 1.5); hold on;
    plot(time_cam, squeeze(y_meas(i,:,1)), 'g.', 'MarkerSize', 6);
    ylabel(sprintf('y_{meas,%d} [m]', i));
    if i == 1
        title('Stereo/Depth Measurement: True vs Noisy (Landmark 1)'); 
        legend('True Relative Pos', 'Measured (20 Hz)', 'Location', 'best'); 
    end
    grid on;
end
xlabel('Time [s]');

disp('Landmark verification complete.');
