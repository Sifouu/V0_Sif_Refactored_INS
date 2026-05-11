% test_generate_trajectory.m
% A script to verify and visualize the output of generate_trajectory.m

clear; clc; close all;
addpath(genpath('../src'));

% Simulation Parameters
frequency = 1000;
t_final = 15; % Increased to 15s to see the full figure-8 shape
type = 'circular';

% Optional: add some noise to test the perturbation struct
% perturbation.pos_noise = 0.05; 
% perturbation.w_noise = 0.01;
perturbation = struct(); % Empty struct for clean, noise-free trajectory

disp('Generating ground truth trajectory...');
tic;
[R_true, P_true, V_true, W_true, A_true] = generate_trajectory(frequency, t_final, type, perturbation);
toc;

time = 0:(1/frequency):t_final;

%% 1. Plot 3D Position Trajectory
figure('Name', '3D Position Trajectory', 'Color', 'w');
plot3(P_true(1,:), P_true(2,:), P_true(3,:), 'b-', 'LineWidth', 2);
grid on; hold on;
% Mark Start and End points
plot3(P_true(1,1), P_true(2,1), P_true(3,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(P_true(1,end), P_true(2,end), P_true(3,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('3D Ground Truth Trajectory (%s)', type));
legend('Path', 'Start', 'End', 'Location', 'best');
view(3);

%% 2. Plot Linear Velocity Components
figure('Name', 'Linear Velocity Components', 'Color', 'w');
subplot(3,1,1);
plot(time, V_true(1,:), 'LineWidth', 1.5);
ylabel('V_x [m/s]'); title('Inertial Linear Velocity'); grid on;

subplot(3,1,2);
plot(time, V_true(2,:), 'LineWidth', 1.5);
ylabel('V_y [m/s]'); grid on;

subplot(3,1,3);
plot(time, V_true(3,:), 'LineWidth', 1.5);
ylabel('V_z [m/s]'); xlabel('Time [s]'); grid on;

%% 3. Plot Angular Velocity Components
figure('Name', 'Angular Velocity Components', 'Color', 'w');
subplot(3,1,1);
plot(time, W_true(1,:), 'LineWidth', 1.5);
ylabel('W_x [rad/s]'); title('Body Angular Velocity'); grid on;

subplot(3,1,2);
plot(time, W_true(2,:), 'LineWidth', 1.5);
ylabel('W_y [rad/s]'); grid on;

subplot(3,1,3);
plot(time, W_true(3,:), 'LineWidth', 1.5);
ylabel('W_z [rad/s]'); xlabel('Time [s]'); grid on;

%% 4. Plot Linear Acceleration Components
figure('Name', 'Linear Acceleration Components', 'Color', 'w');
subplot(3,1,1);
plot(time, A_true(1,:), 'LineWidth', 1.5);
ylabel('A_x [m/s^2]'); title('Inertial Linear Acceleration'); grid on;

subplot(3,1,2);
plot(time, A_true(2,:), 'LineWidth', 1.5);
ylabel('A_y [m/s^2]'); grid on;

subplot(3,1,3);
plot(time, A_true(3,:), 'LineWidth', 1.5);
ylabel('A_z [m/s^2]'); xlabel('Time [s]'); grid on;

%% 5. Verify Rotation Matrices (Sanity Check)
% Check if the final rotation matrix is a valid SO(3) matrix (det = 1, R*R' = I)
R_end = R_true{end};
det_R = det(R_end);
orthogonality_error = norm(R_end * R_end' - eye(3), 'fro');

fprintf('\n--- Sanity Checks ---\n');
fprintf('Final Rotation Matrix Determinant (should be 1): %.6f\n', det_R);
fprintf('Orthogonality Error ||R*R^T - I|| (should be ~0): %e\n', orthogonality_error);
disp('Verification complete. Check the generated figures.');
