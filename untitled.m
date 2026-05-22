% =========================================================================
% Output-Feedback Leader Identification via Unknown Input Observer
% State Space: R^6 (Position and Velocity)
% Measurement: R^3 (Noisy Position Only)
% =========================================================================
clear; clc; close all;

%% 1. Simulation Parameters
dt = 0.02;                  % Sampling time (s)
T_total = 10;               % Total simulation time (s)
steps = T_total / dt;
time = (0:steps-1) * dt;
N = 4;                      % Number of drones (Drone 1 is the Leader)

% Adjacency Matrix (Directed tree: 1->2, 1->3, 2->4)
A_adj = [0 0 0 0; 
         1 0 0 0; 
         1 0 0 0; 
         0 1 0 0];

% Consensus Gains
kp = 3.0; 
kv = 1.5;

% Observer Gains (Pole placement for double integrator)
% Poles at -5, -5 -> s^2 + 10s + 25
L_p = 10 * eye(3); 
L_v = 25 * eye(3);

% Sensor Noise Variance
sigma_v = 0.05; 

% Detection Thresholds
threshold_residual = 1.5; % Threshold for the residual norm to flag a leader
leader_identified_time = NaN;
identified_leader_id = NaN;

%% 2. Initialization
% --- True Plant States ---
p_true = zeros(3, N, steps);
v_true = zeros(3, N, steps);

% Initial positions (Random spread)
p_true(:,:,1) = [0 2 0 4; 0 0 2 2; 5 5 5 5]; % [X; Y; Z] for 4 drones

% --- Observer States ---
p_est = p_true(:,:,1); % Initialize with true first pos for simplicity
v_est = zeros(3, N);
residual_norm = zeros(N, steps);

% Data logging for estimated states
history_p_est = zeros(3, N, steps);
history_v_est = zeros(3, N, steps);

%% 3. Main Simulation Loop
for k = 1:steps-1
    
    % =====================================================================
    % A. THE PLANT (True Adversarial Swarm Dynamics)
    % =====================================================================
    acc_true = zeros(3, N);
    
    % 1. Leader Exogenous Command (Unknown to the Observer)
    % Leader makes a sudden aggressive maneuver between t=3s and t=4.5s
    if time(k) > 3.0 && time(k) < 4.5
        acc_true(:, 1) = [4.0; -2.0; 1.0]; % Sudden acceleration
    end
    
    % 2. Followers compute consensus control based on TRUE states
    for i = 2:N
        u_consensus = zeros(3,1);
        for j = 1:N
            if A_adj(i, j) == 1
                pos_err = p_true(:, j, k) - p_true(:, i, k);
                vel_err = v_true(:, j, k) - v_true(:, i, k);
                u_consensus = u_consensus + kp * pos_err + kv * vel_err;
            end
        end
        acc_true(:, i) = u_consensus;
    end
    
    % 3. Integrate True Plant (Euler)
    v_true(:,:,k+1) = v_true(:,:,k) + acc_true * dt;
    p_true(:,:,k+1) = p_true(:,:,k) + v_true(:,:,k) * dt;
    
    % =====================================================================
    % B. THE SENSOR (Lighthouse / Radar)
    % =====================================================================
    % Add zero-mean Gaussian noise to the true position
    y_meas = p_true(:,:,k+1) + sigma_v * randn(3, N);
    
    % =====================================================================
    % C. THE OBSERVER & DETECTION ALGORITHM
    % =====================================================================
    for i = 1:N
        % 1. Calculate Virtual Consensus Input (Using MEASURED pos and ESTIMATED vel)
        u_virtual = zeros(3,1);
        for j = 1:N
            if A_adj(i, j) == 1
                % Observer assumes distance-based consensus
                pos_err_obs = y_meas(:, j) - y_meas(:, i); 
                vel_err_obs = v_est(:, j) - v_est(:, i);
                u_virtual = u_virtual + kp * pos_err_obs + kv * vel_err_obs;
            end
        end
        
        % 2. Observer Prediction & Update (Luenberger style on double integrator)
        % Innovation/Residual: y - p_hat
        z_residual = y_meas(:, i) - p_est(:, i); 
        
        % State derivatives
        dp_est = v_est(:, i) + L_p * z_residual;
        dv_est = u_virtual + L_v * z_residual;
        
        % Integrate Observer states
        p_est(:, i) = p_est(:, i) + dp_est * dt;
        v_est(:, i) = v_est(:, i) + dv_est * dt;
        
        % 3. Log data
        history_p_est(:, i, k+1) = p_est(:, i);
        history_v_est(:, i, k+1) = v_est(:, i);
        
        % 4. Calculate Detection Metric (Norm of the positional residual)
        % Smooth the residual slightly to prevent noise-induced false positives
        if k > 1
            residual_norm(i, k+1) = 0.8 * residual_norm(i, k) + 0.2 * norm(z_residual);
        else
            residual_norm(i, k+1) = norm(z_residual);
        end
        
        % 5. Leader Identification Logic
        if isnan(leader_identified_time) && residual_norm(i, k+1) > threshold_residual
            leader_identified_time = time(k+1);
            identified_leader_id = i;
        end
    end
end

%% 4. Plotting & Visualization
figure('Name', 'Leader Identification Results', 'Position', [100, 100, 1200, 800]);

% --- Plot 1: 3D Trajectory ---
subplot(2, 2, [1, 3]); hold on; grid on;
colors = lines(N);
for i = 1:N
    plot3(squeeze(p_true(1,i,:)), squeeze(p_true(2,i,:)), squeeze(p_true(3,i,:)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d True', i));
end
% Mark Start and End points
plot3(p_true(1,:,1), p_true(2,:,1), p_true(3,:,1), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k', 'DisplayName', 'Start');
plot3(p_true(1,:,end), p_true(2,:,end), p_true(3,:,end), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'End');
title('Swarm 3D Trajectories');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Location', 'best');
view(3);

% --- Plot 2: Velocity Tracking (X-axis) for the Leader ---
subplot(2, 2, 2); hold on; grid on;
plot(time, squeeze(v_true(1, 1, :)), 'k-', 'LineWidth', 2, 'DisplayName', 'True Vel (Leader)');
plot(time, squeeze(history_v_est(1, 1, :)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Estimated Vel');
title('Observer Performance: Leader X-Axis Velocity');
xlabel('Time (s)'); ylabel('Velocity (m/s)');
legend('Location', 'best');

% --- Plot 3: The Detection Metric (Residuals) ---
subplot(2, 2, 4); hold on; grid on;
for i = 1:N
    plot(time, residual_norm(i, :), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d Innovation', i));
end
% Draw the Threshold Line
yline(threshold_residual, 'k--', 'Threshold $J_{th}$', 'LineWidth', 1.5, 'Interpreter', 'latex', 'LabelHorizontalAlignment', 'left');

% Illustrate the exact moment of detection
if ~isnan(leader_identified_time)
    xline(leader_identified_time, 'r-', 'LineWidth', 2);
    
    % Add a textbox to highlight the identification event
    str = sprintf('LEADER IDENTIFIED!\nDrone %d at t = %.2fs', identified_leader_id, leader_identified_time);
    text(leader_identified_time + 0.2, max(max(residual_norm)) * 0.8, str, ...
        'BackgroundColor', 'white', 'EdgeColor', 'red', 'FontSize', 10, 'FontWeight', 'bold');
end

title('Leader Identification: Innovation Residual Norms');
xlabel('Time (s)'); ylabel('$\| z_i(t) \|$', 'Interpreter', 'latex');
legend('Location', 'northeast');

fprintf('Simulation Complete.\n');
if ~isnan(leader_identified_time)
    fprintf('Algorithm successfully flagged Drone %d as the leader at t = %.2fs.\n', identified_leader_id, leader_identified_time);
else
    fprintf('No leader detected. Consider lowering the threshold.\n');
end