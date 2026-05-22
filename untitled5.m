% =========================================================================
% Output-Feedback Leader Identification via Unknown Input Observer
% UPGRADED: 30s Simulation with High-Performance ANIMATION & Video Export
% =========================================================================
clear; clc; close all;

%% 1. Simulation Parameters & Thresholds (Same as previous)
dt = 0.02;                  
T_total = 30;               
steps = T_total / dt;
time = (0:steps-1) * dt;
N = 4;                      % Drone 1 is the Leader

A_adj = [0 0 0 0; 1 0 0 0; 1 0 0 0; 0 1 0 0];
kp = 3.0; kv = 1.5;
L_p = 10 * eye(3); L_v = 25 * eye(3);
sigma_v = 0.05; 

% Dynamic Threshold Parameters (Tuned)
alpha_filter = 0.015;        
beta_multiplier = 1.8;       
min_threshold = 0.18;        

detection_events = [];      
cooldown_timer = 0;         
cooldown_duration = 2.0;    

%% 2. Initialization & Main Simulation Loop (Fast Calculation)
disp('Simulating Swarm Dynamics...');

p_true = zeros(3, N, steps);
v_true = zeros(3, N, steps);
p_true(:,:,1) = [0 2 0 4; 0 0 2 2; 5 5 5 5]; 
p_est = p_true(:,:,1); 
v_est = zeros(3, N);
residual_norm = zeros(N, steps);
history_v_est = zeros(3, N, steps);
dynamic_threshold = zeros(1, steps);
dynamic_threshold(1) = min_threshold; 
avg_baseline_prev = min_threshold / beta_multiplier;

for k = 1:steps-1
    acc_true = zeros(3, N);
    
    % Maneuvers
    if time(k) > 5.0 && time(k) < 6.5,   acc_true(:, 1) = [6.0; 2.0; 0.0]; end
    if time(k) > 14.0 && time(k) < 16.0, acc_true(:, 1) = [-4.0; 5.0; 4.0]; end
    if time(k) > 23.0 && time(k) < 24.5, acc_true(:, 1) = [-8.0; -6.0; -3.0]; end
    
    % Consensus
    for i = 2:N
        u_consensus = zeros(3,1);
        for j = 1:N
            if A_adj(i, j) == 1
                u_consensus = u_consensus + kp*(p_true(:, j, k) - p_true(:, i, k)) + kv*(v_true(:, j, k) - v_true(:, i, k));
            end
        end
        acc_true(:, i) = u_consensus;
    end
    
    % Physics Integration
    v_true(:,:,k+1) = v_true(:,:,k) + acc_true * dt;
    p_true(:,:,k+1) = p_true(:,:,k) + v_true(:,:,k) * dt;
    y_meas = p_true(:,:,k+1) + sigma_v * randn(3, N);
    
    % Observer
    for i = 1:N
        u_virtual = zeros(3,1);
        for j = 1:N
            if A_adj(i, j) == 1
                u_virtual = u_virtual + kp*(y_meas(:, j) - y_meas(:, i)) + kv*(v_est(:, j) - v_est(:, i));
            end
        end
        z_residual = y_meas(:, i) - p_est(:, i); 
        dp_est = v_est(:, i) + L_p * z_residual;
        dv_est = u_virtual + L_v * z_residual;
        p_est(:, i) = p_est(:, i) + dp_est * dt;
        v_est(:, i) = v_est(:, i) + dv_est * dt;
        history_v_est(:, i, k+1) = v_est(:, i);
        
        if k > 1
            residual_norm(i, k+1) = 0.8 * residual_norm(i, k) + 0.2 * norm(z_residual);
        else
            residual_norm(i, k+1) = norm(z_residual);
        end
    end
    
    % Dynamic Threshold
    current_baseline = median(residual_norm(:, k+1));
    avg_baseline = (1 - alpha_filter) * avg_baseline_prev + alpha_filter * current_baseline;
    avg_baseline_prev = avg_baseline;
    J_th_dynamic = max(beta_multiplier * avg_baseline, min_threshold);
    dynamic_threshold(k+1) = J_th_dynamic;
    
    % Detection
    if residual_norm(1, k+1) > J_th_dynamic && cooldown_timer <= 0
        detection_events = [detection_events, time(k+1)]; 
        cooldown_timer = cooldown_duration; 
    end
    if cooldown_timer > 0, cooldown_timer = cooldown_timer - dt; end
end

%% 3. ANIMATION SETUP
disp('Initializing Animation...');

% Setup Video Writer (Optional: Change 'false' to 'true' to save an MP4)
save_video = false; 
if save_video
    v = VideoWriter('Swarm_Leader_Detection.mp4', 'MPEG-4');
    v.FrameRate = 30; % 30 FPS
    open(v);
end

fig = figure('Name', 'Swarm Animation', 'Position', [100, 50, 1400, 800], 'Color', 'w');
colors = lines(N);

% --- Subplot 1: 3D Trajectory Setup ---
subplot(2, 2, [1, 3]); hold on; grid on; view(3);
title('Live Swarm 3D Trajectories', 'FontSize', 12);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% Lock axes limits so the camera doesn't jump around
xlim([min(p_true(1,:,:),[],'all')-5, max(p_true(1,:,:),[],'all')+5]);
ylim([min(p_true(2,:,:),[],'all')-5, max(p_true(2,:,:),[],'all')+5]);
zlim([min(p_true(3,:,:),[],'all')-5, max(p_true(3,:,:),[],'all')+5]);

% Initialize 3D graphical objects (Empty arrays)
for i = 1:N
    h_traj(i) = plot3(NaN, NaN, NaN, 'Color', [colors(i,:) 0.5], 'LineWidth', 1.5, 'DisplayName', sprintf('Drone %d', i));
    h_drone(i) = plot3(NaN, NaN, NaN, 'o', 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
end
legend(h_traj, 'Location', 'best');

% --- Subplot 2: Velocity Setup ---
subplot(2, 2, 2); hold on; grid on;
title('Live Observer X-Axis Velocity Profile', 'FontSize', 12);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
xlim([0 T_total]); ylim([min(v_true(1,1,:))-2, max(v_true(1,1,:))+2]);
h_vel_true = plot(NaN, NaN, 'k-', 'LineWidth', 2, 'DisplayName', 'True Vel (Leader)');
h_vel_est  = plot(NaN, NaN, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Estimated Vel');
legend('Location', 'best');

% --- Subplot 3: Residuals Setup ---
subplot(2, 2, 4); hold on; grid on;
title('Live Leader Identification (Dynamic Threshold)', 'FontSize', 12);
xlabel('Time (s)'); ylabel('$\| z_i(t) \|$', 'Interpreter', 'latex');
xlim([0 T_total]); ylim([0, max(max(residual_norm)) + 0.2]);

for i = 1:N
    h_res(i) = plot(NaN, NaN, 'Color', colors(i,:), 'LineWidth', 1.5);
end
h_thresh = plot(NaN, NaN, 'k--', 'LineWidth', 2, 'DisplayName', 'Dynamic Threshold $J_{th}(t)$');

% Time indicator line
h_time_line = xline(0, 'g-', 'LineWidth', 1.5, 'Alpha', 0.5); 

%% 4. ANIMATION EXECUTION LOOP
disp('Playing Animation...');
playback_speed = 3; % Skip frames to speed up playback (e.g., plot every 3rd calculation step)
next_event_idx = 1;

for k = 1 : playback_speed : steps
    current_time = time(k);
    
    % Update 3D Trajectories (Draw up to current time 'k')
    for i = 1:N
        set(h_traj(i), 'XData', squeeze(p_true(1, i, 1:k)), ...
                       'YData', squeeze(p_true(2, i, 1:k)), ...
                       'ZData', squeeze(p_true(3, i, 1:k)));
                   
        % Move the drone marker to the current tip of the trajectory
        set(h_drone(i), 'XData', p_true(1, i, k), ...
                        'YData', p_true(2, i, k), ...
                        'ZData', p_true(3, i, k));
    end
    
    % Update Velocity Plot
    set(h_vel_true, 'XData', time(1:k), 'YData', squeeze(v_true(1, 1, 1:k)));
    set(h_vel_est,  'XData', time(1:k), 'YData', squeeze(history_v_est(1, 1, 1:k)));
    
    % Update Residuals and Threshold Plot
    for i = 1:N
        set(h_res(i), 'XData', time(1:k), 'YData', residual_norm(i, 1:k));
    end
    set(h_thresh, 'XData', time(1:k), 'YData', dynamic_threshold(1:k));
    
    % Move the green time tracker line
    set(h_time_line, 'Value', current_time);
    
    % Check if an evasion event occurred and place a permanent marker
    if next_event_idx <= length(detection_events) && current_time >= detection_events(next_event_idx)
        t_ev = detection_events(next_event_idx);
        % Draw on Subplot 3
        subplot(2, 2, 4);
        xline(t_ev, 'r-', 'LineWidth', 2);
        str = sprintf('EVASION %d', next_event_idx);
        text(t_ev + 0.3, max(max(residual_norm)) * 0.8, str, 'BackgroundColor', 'white', 'EdgeColor', 'red', 'FontSize', 9, 'FontWeight', 'bold');
        
        % Optionally mark the exact 3D location of the evasion
        subplot(2, 2, [1, 3]);
        plot3(p_true(1,1,k), p_true(2,1,k), p_true(3,1,k), 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
        
        next_event_idx = next_event_idx + 1;
    end
    
    % Render the updated graphics
    drawnow;
    
    % Save to video if enabled
    if save_video
        frame = getframe(fig);
        writeVideo(v, frame);
    end
end

if save_video
    close(v);
    disp('Animation saved as Swarm_Leader_Detection.mp4');
else
    disp('Animation complete.');
end