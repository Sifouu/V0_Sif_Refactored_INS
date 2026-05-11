function estimates = generic_observer(measurements, init_state, params)
% GENERIC_OBSERVER Implements a Riccati-based LPV observer for INS.
% Based on a pseudo-linear formulation with a Nonlinear Complementary Filter (NCF).

    %% 1. Initialization
    time_imu = measurements.imu.time;
    N = length(time_imu);
    
    % Preallocate outputs
    estimates.P = zeros(3, N);
    estimates.V = zeros(3, N);
    estimates.R = cell(1, N);
    estimates.time = time_imu;
    
    % Initialize Rotation Matrix
    current_R = init_state.R;
    
    % Assemble 15x1 state vector x = [p_B; v_B; e_B1; e_B2; e_B3]
    current_P_B = current_R' * init_state.P;
    current_V_B = current_R' * init_state.V;
    e_B1 = current_R' * [1; 0; 0];
    e_B2 = current_R' * [0; 1; 0];
    e_B3 = current_R' * [0; 0; 1];
    x = [current_P_B; current_V_B; e_B1; e_B2; e_B3];
    
    % Initialize Riccati Covariance Matrix
    P_cov = params.P0;
    
    % Exact discrete matrices for LPV
    A_bar = zeros(15);
    A_bar(1:3, 4:6) = eye(3);
    A_bar(4:6, 13:15) = params.g * eye(3); % Gravity term coupled with e_B3
    
    B_matrix = zeros(15, 3);
    B_matrix(4:6, :) = eye(3);
    
    % Save initial estimates
    estimates.P(:, 1) = init_state.P;
    estimates.V(:, 1) = init_state.V;
    estimates.R{1}    = init_state.R;
    
    %% 2. Main Estimation Loop
    for i = 2:N
        t = time_imu(i);
        dt = t - time_imu(i-1);
        
        % Current IMU readings
        a_B = measurements.imu.a_B(:, i-1);
        w_B = measurements.imu.w_B(:, i-1);
        
        % Form LPV state matrix A
        w_x = ALLFUNCS.skew(w_B);
        S_w = -blkdiag(w_x, w_x, w_x, w_x, w_x);
        A = A_bar + S_w;
        
        % Initialize Dynamic Measurement Matrices
        C_dyn = [];
        y_bar_dyn = [];
        Q_dyn = [];
        
        % --- A. CHECK GPS MEASUREMENTS ---
        if isfield(measurements, 'gps')
            % Find closest GPS measurement
            [dt_diff, idx_gps] = min(abs(measurements.gps.time - t));
            % Check if we hit a GPS measurement exactly and it is valid
            if dt_diff < 1e-4 && measurements.gps.is_valid(idx_gps)
                p_gps = measurements.gps.p_meas(:, idx_gps);
                
                % C matrix block for GPS position: kron([-1 0 p_gps'], I3)
                C_gps = kron([-1, 0, p_gps'], eye(3));
                y_bar_gps = zeros(3, 1);
                
                C_dyn = [C_dyn; C_gps];
                y_bar_dyn = [y_bar_dyn; y_bar_gps];
                Q_dyn = blkdiag(Q_dyn, params.Q_gps);
            end
        end
        
        % --- B. CHECK LANDMARK MEASUREMENTS ---
        if isfield(measurements, 'cam')
            [dt_diff, idx_cam] = min(abs(measurements.cam.time - t));
            if dt_diff < 1e-4
                num_landmarks = size(measurements.cam.positions_I, 2);
                for l = 1:num_landmarks
                    p_land_I = measurements.cam.positions_I(:, l);
                    y_meas_land = measurements.cam.landmark_meas(:, idx_cam, l);
                    
                    % C matrix block for Landmark
                    C_land = kron([-1, 0, p_land_I'], eye(3));
                    
                    C_dyn = [C_dyn; C_land];
                    y_bar_dyn = [y_bar_dyn; y_meas_land];
                    Q_dyn = blkdiag(Q_dyn, params.Q_cam);
                end
            end
        end
        
        % --- C. CONTINUOUS RICCATI INTEGRATION ---
        if ~isempty(C_dyn)
            % Compute Kalman-like Gain
            K = P_cov * C_dyn' * Q_dyn;
            
            % Update Covariance
            P_dot = A * P_cov + P_cov * A' - P_cov * C_dyn' * Q_dyn * C_dyn * P_cov + params.V_noise;
            P_cov = P_cov + dt * P_dot;
            P_cov = 0.5 * (P_cov + P_cov'); % Ensure symmetry
            
            % Update State
            x_dot = A * x + B_matrix * a_B + K * (y_bar_dyn - C_dyn * x);
            x = x + dt * x_dot;
        else
            % Dead-reckoning (No valid measurements)
            P_dot = A * P_cov + P_cov * A' + params.V_noise;
            P_cov = P_cov + dt * P_dot;
            P_cov = 0.5 * (P_cov + P_cov'); % Ensure symmetry
            
            x_dot = A * x + B_matrix * a_B;
            x = x + dt * x_dot;
        end
        
        % Extract Auxiliary States
        e_B1 = x(7:9);
        e_B2 = x(10:12);
        e_B3 = x(13:15);
        
        % --- D. ATTITUDE RECONSTRUCTION (NCF) ---
        sigma = cross(e_B1, current_R' * [1; 0; 0]) + ...
                cross(e_B2, current_R' * [0; 1; 0]) + ...
                cross(e_B3, current_R' * [0; 0; 1]);
                
        w_total = w_B + params.k_att * sigma;
        current_R = current_R * ALLFUNCS.Rexp(w_total * dt);
        
        % Ensure rotation matrix remains proper SO(3)
        current_R = ALLFUNCS.orthogonalize(current_R);
        
        % --- E. STORE ESTIMATES ---
        estimates.P(:, i) = current_R * x(1:3);
        estimates.V(:, i) = current_R * x(4:6);
        estimates.R{i}    = current_R;
    end
end
