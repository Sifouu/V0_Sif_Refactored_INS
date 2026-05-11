% test_simulate_gps.m
% Script to verify the GPS simulation function (downsampling, noise, outages)

clear; clc; close all;
addpath(genpath('../src'));

%% 1. Generate High-Frequency Ground Truth
frequency_gt = 1000; % 1000 Hz Ground truth
t_final = 15;
type = 'figure8';
perturbation = struct();

disp('Generating ground truth (1000 Hz)...');
[~, P_true, V_true, ~, ~] = generate_trajectory(frequency_gt, t_final, type, perturbation);
time_gt = 0:(1/frequency_gt):t_final;

%% 2. Setup GPS Parameters
% Downsample to 5 Hz
gps_params.frequency = 5; 
gps_params.noise_std_pos = 1.0; % 1m position noise
gps_params.noise_std_vel = 0.1; % 0.1 m/s velocity noise

% Simulate a 4-second GPS outage in the middle of the flight
gps_params.outage_start = 5.0;
gps_params.outage_end = 9.0;

%% 3. Simulate GPS
disp('Simulating GPS measurements (5 Hz)...');
tic;
[p_gps_meas, v_gps_meas, is_valid, time_gps] = simulate_gps(P_true, V_true, time_gt, gps_params);
toc;

%% 4. Plot Results

% Figure 1: 3D Position with Outage
figure('Name', 'GPS 3D Trajectory', 'Color', 'w');
plot3(P_true(1,:), P_true(2,:), P_true(3,:), 'k-', 'LineWidth', 1.5); hold on;
% Plot only valid GPS points
plot3(p_gps_meas(1, is_valid), p_gps_meas(2, is_valid), p_gps_meas(3, is_valid), ...
      'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 4);
grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Trajectory showing GPS Outage between 5s and 9s');
legend('True Trajectory', 'Valid GPS Measurements', 'Location', 'best');
view(3);

% Figure 2: Position Components vs Time
figure('Name', 'GPS Position Components', 'Color', 'w');
for i = 1:3
    subplot(3,1,i);
    plot(time_gt, P_true(i,:), 'k-', 'LineWidth', 1.5); hold on;
    % The NaN values will automatically create a gap in the plot
    plot(time_gps, p_gps_meas(i,:), 'b.', 'MarkerSize', 8);
    
    % Highlight the outage region
    yl = ylim;
    patch([gps_params.outage_start gps_params.outage_end gps_params.outage_end gps_params.outage_start], ...
          [yl(1) yl(1) yl(2) yl(2)], 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
          
    ylabel(sprintf('P_{%d} [m]', i));
    if i == 1
        title('GPS Position vs Truth (Red shaded = Outage)'); 
        legend('Truth', 'Measured (5 Hz)', 'Outage', 'Location', 'best'); 
    end
    grid on;
end
xlabel('Time [s]');

% Figure 3: Velocity Components vs Time
figure('Name', 'GPS Velocity Components', 'Color', 'w');
for i = 1:3
    subplot(3,1,i);
    plot(time_gt, V_true(i,:), 'k-', 'LineWidth', 1.5); hold on;
    % The NaN values will automatically create a gap in the plot
    plot(time_gps, v_gps_meas(i,:), 'g.', 'MarkerSize', 8);
    
    % Highlight the outage region
    yl = ylim;
    patch([gps_params.outage_start gps_params.outage_end gps_params.outage_end gps_params.outage_start], ...
          [yl(1) yl(1) yl(2) yl(2)], 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
          
    ylabel(sprintf('V_{%d} [m/s]', i));
    if i == 1
        title('GPS Velocity vs Truth (Red shaded = Outage)'); 
        legend('Truth', 'Measured (5 Hz)', 'Outage', 'Location', 'best'); 
    end
    grid on;
end
xlabel('Time [s]');

disp('GPS verification complete.');
