% =========================================================================
% Output-Feedback Leader Identification via Unknown Input Observer
% UPGRADED: Extended Time, Spaced Aggressive Maneuvers, Continuous Detection
% =========================================================================
clear; clc; close all;

%% 1. Simulation Parameters
dt = 0.02;                  
T_total = 30;               % INCREASED: 30-second simulation
steps = T_total / dt;
time = (0:steps-1) * dt;
N = 4;                      % Drone 1 is the Leader

% Adjacency Matrix (Directed tree: 1->2, 1->3, 2->4)
A_adj = [0 0 0 0; 
         1 0 0 0; 
         1 0 0 0; 
         0 1 0 0];

% Consensus Gains
kp = 3.0; 
kv = 1.5;

% Observer Gains 
L_p = 10 * eye(3); 
L_v = 25 * eye(3);

% Sensor Noise Variance
sigma_v = 0.05; 

% Multi-Level Detection Thresholds
threshold_sensitive = 0.10; 
threshold_strict    = 0.25; 

% Event tracking for continuous detection
detection_events = [];      % Store all timestamps of aggressive maneuvers
cooldown_timer = 0;         % Prevent multiple triggers for the same maneuver
cooldown_duration = 2.0;    % 2 seconds before the system can flag a NEW maneuver

%% 2. Initialization
p_true = zeros(3, N, steps);
v_true = zeros(3, N, steps);

% Initial positions
p_true(:,:,1) = [0 2 0 4; 0 0 2 2; 5 5 5 5]; 

p_est = p_true(:,:,1); 
v_est = zeros(3, N);
residual_norm = zeros(N, steps);

history_p_est = zeros(3, N, steps);
history_v_est = zeros(3, N, steps);

%% 3. Main Simulation Loop
for k = 1:steps-1
    
    % =====================================================================
    % A. THE PLANT (True Adversarial Swarm Dynamics)
    % =====================================================================
    acc_true = zeros(3, N);
    
    % --- EXTENDED MANEUVER PROFILES ---
    % Maneuver 1: High-Speed Forward Dash
    if time(k) > 5.0 && time(k) < 6.5
        acc_true(:, 1) = [6.0; 2.0; 0.0]; 
    end
    
    % Maneuver 2: Sharp Evasive Climb & Turn
    if time(k) > 14.0 && time(k) < 16.0
        acc_true(:, 1) = [-4.0; 5.0; 4.0]; 
    end
    
    % Maneuver 3: Sudden Aggressive Reversal (Braking and retreating)
    if time(k) > 23.0 && time(k) < 24.5
        acc_true(:, 1) = [-8.0; -6.0; -3.0]; 
    end
    
    % Followers compute consensus control
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
    
    % Integrate True Plant
    v_true(:,:,k+1) = v_true(:,:,k) + acc_true * dt;
    p_true(:,:,k+1) = p_true(:,:,k) + v_true(:,:,k) * dt;
    
    % =====================================================================
    % B. THE SENSOR (Lighthouse / Radar)
    % =====================================================================
    y_meas = p_true(:,:,k+1) + sigma_v * randn(3, N);
    
    % =====================================================================
    % C. THE OBSERVER & DETECTION ALGORITHM
    % =====================================================================
    for i = 1:N
        u_virtual = zeros(3,1);
        for j = 1:N
            if A_adj(i, j) == 1
                pos_err_obs = y_meas(:, j) - y_meas(:, i); 
                vel_err_obs = v_est(:, j) - v_est(:, i);
                u_virtual = u_virtual + kp * pos_err_obs + kv * vel_err_obs;
            end
        end
        
        z_residual = y_meas(:, i) - p_est(:, i); 
        
        dp_est = v_est(:, i) + L_p * z_residual;
        dv_est = u_virtual + L_v * z_residual;
        
        p_est(:, i) = p_est(:, i) + dp_est * dt;
        v_est(:, i) = v_est(:, i) + dv_est * dt;
        
        history_p_est(:, i, k+1) = p_est(:, i);
        history_v_est(:, i, k+1) = v_est(:, i);
        
        % Smooth the residual (EMA filter)
        if k > 1
            residual_norm(i, k+1) = 0.8 * residual_norm(i, k) + 0.2 * norm(z_residual);
        else
            residual_norm(i, k+1) = norm(z_residual);
        end
        
        % Continuous Leader Identification Logic
        if i == 1 % We know Drone 1 is the leader for tracking events
            if cooldown_timer > 0
                cooldown_timer = cooldown_timer - dt;
            elseif residual_norm(i, k+1) > threshold_strict
                detection_events = [detection_events, time(k+1)]; % Log the event
                cooldown_timer = cooldown_duration; % Reset the cooldown
            end
        end
    end
end

%% 4. Plotting & Visualization
figure('Name', 'Extended Swarm Leader Identification', 'Position', [100, 100, 1200, 800]);

% --- Plot 1: 3D Trajectory ---
subplot(2, 2, [1, 3]); hold on; grid on;
colors = lines(N);
for i = 1:N
    plot3(squeeze(p_true(1,i,:)), squeeze(p_true(2,i,:)), squeeze(p_true(3,i,:)), ...
        'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d True', i));
end
plot3(p_true(1,:,1), p_true(2,:,1), p_true(3,:,1), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k', 'DisplayName', 'Start');
plot3(p_true(1,:,end), p_true(2,:,end), p_true(3,:,end), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'End');
title('30-Second Swarm 3D Trajectories (Three Maneuvers)');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Location', 'best');
view(3);

% --- Plot 2: Velocity Tracking (X-axis) ---
subplot(2, 2, 2); hold on; grid on;
plot(time, squeeze(v_true(1, 1, :)), 'k-', 'LineWidth', 2, 'DisplayName', 'True Vel (Leader)');
plot(time, squeeze(history_v_est(1, 1, :)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Estimated Vel');
title('Observer: Leader X-Axis Velocity Profile');
xlabel('Time (s)'); ylabel('Velocity (m/s)');
legend('Location', 'best');

% --- Plot 3: The Detection Metric (Multi-Thresholds) ---
subplot(2, 2, 4); hold on; grid on;
for i = 1:N
    plot(time, residual_norm(i, :), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d Innovation', i));
end

% Draw the Sensitive and Strict Threshold Lines
yline(threshold_strict, 'k--', 'Strict Threshold ($J_{strict}$)', 'LineWidth', 1.5, 'Interpreter', 'latex', 'LabelHorizontalAlignment', 'left');
yline(threshold_sensitive, 'k:', 'Sensitive Threshold ($J_{sens}$)', 'LineWidth', 1.5, 'Interpreter', 'latex', 'LabelHorizontalAlignment', 'left');

% Illustrate ALL moments of detection
for ev = 1:length(detection_events)
    t_ev = detection_events(ev);
    xline(t_ev, 'r-', 'LineWidth', 2);
    str = sprintf('EVASION %d', ev);
    text(t_ev + 0.5, threshold_strict + 0.15, str, ...
        'BackgroundColor', 'white', 'EdgeColor', 'red', 'FontSize', 9, 'FontWeight', 'bold');
end

title('Continuous Identification: Innovation Residuals over 30s');
xlabel('Time (s)'); ylabel('$\| z_i(t) \|$', 'Interpreter', 'latex');
legend('Location', 'northeast');
ylim([0, max(max(residual_norm)) + 0.3]); % Auto-scale to fit the highest peak safely