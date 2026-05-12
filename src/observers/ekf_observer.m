function estimates = ekf_observer(measurements, init_state, params)
% EKF_OBSERVER Implements a standard Error-State Extended Kalman Filter for INS.
%
% This uses a 9-DOF error state vector: dx = [dp; dv; dtheta]
% and Left-Invariant error definition: R = exp(skew(dtheta)) * R_hat

    %% 1. Initialization
    time_imu = measurements.imu.time;
    N = length(time_imu);
    
    % Preallocate outputs
    estimates.P = zeros(3, N);
    estimates.V = zeros(3, N);
    estimates.R = cell(1, N);
    estimates.time = time_imu;
    
    % Initialize Nominal State
    hat_p = init_state.P;
    hat_v = init_state.V;
    hat_R = init_state.R;
    
    % Initialize Error State Covariance Matrix (9x9)
    P_cov = blkdiag(params.P0(1:3,1:3), params.P0(4:6,4:6), params.P0(7:9,7:9));
    
    % Define Process Noise (IMU variances)
    Q_a = params.Q_imu_accel;  % Accelerometer noise
    Q_w = params.Q_imu_gyro; % Gyroscope noise
    
    % Save initial estimates
    estimates.P(:, 1) = hat_p;
    estimates.V(:, 1) = hat_v;
    estimates.R{1}    = hat_R;
    
    %% 2. Main Estimation Loop
    for i = 2:N
        t = time_imu(i);
        dt = t - time_imu(i-1);
        
        % Current IMU readings
        a_B = measurements.imu.a_B(:, i-1);
        w_B = measurements.imu.w_B(:, i-1);
        
        % --- A. PROPAGATION (PREDICTION) ---
        % 1. Propagate Nominal State
        hat_p = hat_p + hat_v * dt;
        hat_v = hat_v + (hat_R * a_B + params.g_vec) * dt;
        hat_R = hat_R * ALLFUNCS.Rexp(w_B * dt);
        
        % 2. Propagate Covariance
        % Continuous Jacobian F
        F = zeros(9, 9);
        F(1:3, 4:6) = eye(3);
        F(4:6, 7:9) = -ALLFUNCS.skew(hat_R * a_B);
        
        % Discrete transition matrix (Euler integration)
        Phi = eye(9) + F * dt;
        
        % Map IMU noise into state space
        G = zeros(9, 6);
        G(4:6, 1:3) = hat_R;
        G(7:9, 4:6) = hat_R;
        Q_c = G * blkdiag(Q_a, Q_w) * G';
        Q_d = Q_c * dt;
        
        P_cov = Phi * P_cov * Phi' + Q_d;
        P_cov = 0.5 * (P_cov + P_cov'); % Ensure symmetry
        
        % Initialize Update Variables
        H_total = [];
        z_total = [];
        R_total = [];
        
        % --- B. GPS UPDATE ---
        if isfield(measurements, 'gps')
            [dt_diff, idx_gps] = min(abs(measurements.gps.time - t));
            % Check if we hit a GPS measurement exactly and it is valid
            if dt_diff < 1e-4 && measurements.gps.is_valid(idx_gps)
                p_gps = measurements.gps.p_meas(:, idx_gps);
                
                % Innovation
                z_gps = p_gps - hat_p;
                % Jacobian
                H_gps = [eye(3), zeros(3, 3), zeros(3, 3)];
                
                H_total = [H_total; H_gps];
                z_total = [z_total; z_gps];
                R_total = blkdiag(R_total, params.Q_gps);
            end
        end
        
        % --- C. LANDMARK UPDATE ---
        if isfield(measurements, 'cam')
            [dt_diff, idx_cam] = min(abs(measurements.cam.time - t));
            if dt_diff < 1e-4
                num_landmarks = size(measurements.cam.positions_I, 2);
                for l = 1:num_landmarks
                    p_land_I = measurements.cam.positions_I(:, l);
                    y_meas_land = measurements.cam.landmark_meas(:, idx_cam, l); % relative pos in body frame
                    
                    % Predicted measurement
                    hat_y = hat_R' * (p_land_I - hat_p);
                    
                    % Innovation
                    z_cam = y_meas_land - hat_y;
                    
                    % Jacobian
                    H_cam = [-hat_R', zeros(3,3), hat_R' * ALLFUNCS.skew(p_land_I - hat_p)];
                    
                    H_total = [H_total; H_cam];
                    z_total = [z_total; z_cam];
                    R_total = blkdiag(R_total, params.Q_cam);
                end
            end
        end
        
        % --- D. COMPUTE CORRECTION ---
        if ~isempty(H_total)
            % Compute Kalman Gain
            S = H_total * P_cov * H_total' + R_total;
            K = P_cov * H_total' / S;
            
            % Compute error state correction
            dx = K * z_total;
            dp = dx(1:3);
            dv = dx(4:6);
            dtheta = dx(7:9);
            
            % Inject error into nominal state (Reset)
            hat_p = hat_p + dp;
            hat_v = hat_v + dv;
            hat_R = ALLFUNCS.Rexp(dtheta) * hat_R;
            hat_R = ALLFUNCS.orthogonalize(hat_R);
            
            % Update Covariance (Joseph form for numerical stability)
            I_KH = eye(9) - K * H_total;
            P_cov = I_KH * P_cov * I_KH' + K * R_total * K';
            P_cov = 0.5 * (P_cov + P_cov');
        end
        
        % --- E. STORE ESTIMATES ---
        estimates.P(:, i) = hat_p;
        estimates.V(:, i) = hat_v;
        estimates.R{i}    = hat_R;
    end
end

