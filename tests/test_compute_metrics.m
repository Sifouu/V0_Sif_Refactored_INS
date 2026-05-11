% test_compute_metrics.m
% Script to verify the metric computation function

clear; clc; close all;
addpath('src/evaluation');

%% 1. Generate Fake Ground Truth
N = 100;
ground_truth.P = randn(3, N);
ground_truth.V = randn(3, N);
ground_truth.R = cell(1, N);
for i = 1:N
    ground_truth.R{i} = eye(3); % Keep true attitude as Identity for simplicity
end

%% 2. Generate Fake Estimates with Known Errors
estimates.P = zeros(3, N);
estimates.V = zeros(3, N);
estimates.R = cell(1, N);

theta = 0.1; % Constant 0.1 radian rotation error

for i = 1:N
    % Inject a constant 1-meter error in X for position
    estimates.P(:, i) = ground_truth.P(:, i) + [1; 0; 0];
    
    % Inject a constant 0.5 m/s error in Y for velocity
    estimates.V(:, i) = ground_truth.V(:, i) + [0; 0.5; 0];
    
    % Inject a constant rotation error (around Z axis)
    R_err = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    estimates.R{i} = ground_truth.R{i} * R_err;
end

%% 3. Compute Metrics
disp('Computing metrics...');
tic;
[errors, rmse] = compute_metrics(ground_truth, estimates);
toc;

%% 4. Verify Results
% Since R_true is Identity, the geometric error for position will be:
% || I*P_true - R_est'*P_est ||. Because R_est = R_err, this evaluates the coupled error.
expected_trace_err = 1 - cos(theta);

fprintf('\n--- Verification Checks ---\n');
fprintf('Position Euclidean RMSE: %.4f (Expected: 1.0000)\n', rmse.position_euclidean);
fprintf('Velocity Euclidean RMSE: %.4f (Expected: 0.5000)\n', rmse.velocity_euclidean);
fprintf('Attitude Trace RMSE:     %.6f (Expected: %.6f)\n', rmse.attitude_trace, expected_trace_err);

%% 5. Plot the time-series errors
time = 1:N;

figure('Name', 'Estimation Errors', 'Color', 'w');
subplot(3,1,1);
plot(time, errors.pos_euclidean, 'r-', 'LineWidth', 2); hold on;
plot(time, errors.pos_geometric, 'b--', 'LineWidth', 1.5);
ylabel('Pos Error [m]'); title('Position Errors'); legend('Euclidean', 'Geometric', 'Location', 'best'); grid on;

subplot(3,1,2);
plot(time, errors.vel_euclidean, 'r-', 'LineWidth', 2); hold on;
plot(time, errors.vel_geometric, 'b--', 'LineWidth', 1.5);
ylabel('Vel Error [m/s]'); title('Velocity Errors'); legend('Euclidean', 'Geometric', 'Location', 'best'); grid on;

subplot(3,1,3);
plot(time, errors.att_trace, 'k-', 'LineWidth', 2);
ylabel('Trace Error'); xlabel('Time step'); title('Attitude Error (Trace)'); grid on;

disp('Metrics verification complete.');
