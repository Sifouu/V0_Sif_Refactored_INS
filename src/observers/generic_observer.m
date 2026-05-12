
function estimates = generic_observer(measurements, init_state, params)
% GENERIC_OBSERVER Implements a Riccati-based LPV observer for INS.
% Based on a pseudo-linear formulation with a Nonlinear Complementary Filter (NCF).
% Updated to use a Continuous-Discrete Hybrid architecture for software stability.

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
    
    Q_a = params.Q_imu_accel;  % Accelerometer noise
    Q_w = params.Q_imu_gyro; % Gyroscope noise


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
            % Ensure the GPS sample is effectively synchronized with current IMU time
            % and that the GPS measurement at that index is marked valid.
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

        % --- C. CONTINUOUS-DISCRETE INTEGRATION (HYBRID) ---

        % 1. CONTINUOUS PREDICTION (Runs every IMU cycle)
        % Predict the state and covariance forward by dt

        G = zeros(15, 6);
        G(4:6, 1:3) = current_R;
        G(7:9, 4:6) = current_R;
        Q_c = G * blkdiag(Q_a, Q_w) * G';

        P_dot = A * P_cov + P_cov * A' + Q_c;
        P_cov_pred = P_cov + dt * P_dot;
        P_cov_pred = 0.5 * (P_cov_pred + P_cov_pred'); % Ensure symmetry

        x_dot_pred = A * x + B_matrix * a_B;
        x_pred = x + dt * x_dot_pred;

        % 2. DISCRETE UPDATE (Runs ONLY when a measurement hits)
        if ~isempty(C_dyn)

            % Standard Discrete Kalman Gain (No inversion needed)
            S = C_dyn * P_cov_pred * C_dyn' + Q_dyn;
            K = P_cov_pred * C_dyn' / S;

            % Discrete State Update (Instant jump, NOT multiplied by dt)
            x = x_pred + K * (y_bar_dyn - C_dyn * x_pred);

            % Discrete Covariance Update (Joseph form for numerical stability)
            I_KC = eye(15) - K * C_dyn;
            P_cov = I_KC * P_cov_pred * I_KC' + K * Q_dyn * K';
            P_cov = 0.5 * (P_cov + P_cov'); 

        else
            % No measurements: lock in the predictions
            x = x_pred;
            P_cov = P_cov_pred;
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



% 
% function estimates = generic_observer(measurements, init_state, params)
% % GENERIC_OBSERVER Implements a Riccati-based LPV observer for INS.
% % Based on a pseudo-linear formulation with a Nonlinear Complementary Filter (NCF).
% % Updated to use an EXACT Analytical Discrete Prediction and Hybrid Update.
% 
%     %% 1. Initialization
%     time_imu = measurements.imu.time;
%     N = length(time_imu);
% 
%     % Preallocate outputs
%     estimates.P = zeros(3, N);
%     estimates.V = zeros(3, N);
%     estimates.R = cell(1, N);
%     estimates.time = time_imu;
% 
%     % Initialize Rotation Matrix
%     current_R = init_state.R;
% 
%     % Assemble 15x1 state vector x = [p_B; v_B; e_B1; e_B2; e_B3]
%     current_P_B = current_R' * init_state.P;
%     current_V_B = current_R' * init_state.V;
%     e_B1 = current_R' * [1; 0; 0];
%     e_B2 = current_R' * [0; 1; 0];
%     e_B3 = current_R' * [0; 0; 1];
%     x = [current_P_B; current_V_B; e_B1; e_B2; e_B3];
% 
%     % Initialize Riccati Covariance Matrix
%     P_cov = params.P0;
% 
%     % Exact discrete matrices for LPV
%     A_bar = zeros(15);
%     A_bar(1:3, 4:6) = eye(3);
%     A_bar(4:6, 13:15) = params.g * eye(3); % Gravity term coupled with e_B3
% 
%     B_matrix = zeros(15, 3);
%     B_matrix(4:6, :) = eye(3);
% 
%     % Save initial estimates
%     estimates.P(:, 1) = init_state.P;
%     estimates.V(:, 1) = init_state.V;
%     estimates.R{1}    = init_state.R;
%     m = 0;
% 
%     %% 2. Main Estimation Loop
%     for i = 2:N
%         t = time_imu(i);
%         dt = t - time_imu(i-1);
% 
%         % Current IMU readings
%         a_B = measurements.imu.a_B(:, i-1);
%         w_B = measurements.imu.w_B(:, i-1);
% 
%         % Initialize Dynamic Measurement Matrices
%         C_dyn = [];
%         y_bar_dyn = [];
%         Q_dyn = [];
% 
%         % --- A. CHECK GPS MEASUREMENTS ---
%         if isfield(measurements, 'gps')
%             [dt_diff, idx_gps] = min(abs(measurements.gps.time - t));
%             if dt_diff < 1e-4 && measurements.gps.is_valid(idx_gps)
%                 m = m + 1;
%                 p_gps = measurements.gps.p_meas(:, idx_gps);
% 
%                 % C matrix block for GPS position: kron([-1 0 p_gps'], I3)
%                 C_gps = kron([-1, 0, p_gps'], eye(3));
%                 y_bar_gps = zeros(3, 1);
% 
%                 C_dyn = [C_dyn; C_gps];
%                 y_bar_dyn = [y_bar_dyn; y_bar_gps];
%                 Q_dyn = blkdiag(Q_dyn, params.Q_gps);
%             end
%         end
% 
%         % --- B. CHECK LANDMARK MEASUREMENTS ---
%         if isfield(measurements, 'cam')
%             [dt_diff, idx_cam] = min(abs(measurements.cam.time - t));
%             if dt_diff < 1e-4
%                 num_landmarks = size(measurements.cam.positions_I, 2);
%                 for l = 1:num_landmarks
%                     p_land_I = measurements.cam.positions_I(:, l);
%                     y_meas_land = measurements.cam.landmark_meas(:, idx_cam, l);
% 
%                     % C matrix block for Landmark
%                     C_land = kron([-1, 0, p_land_I'], eye(3));
% 
%                     C_dyn = [C_dyn; C_land];
%                     y_bar_dyn = [y_bar_dyn; y_meas_land];
%                     Q_dyn = blkdiag(Q_dyn, params.Q_cam);
%                 end
%             end
%         end
% 
%         % --- C. EXACT DISCRETE PREDICTION & HYBRID UPDATE ---
% 
%         % 1. EXACT DISCRETE PREDICTION (Runs every IMU cycle)
%         % Computed analytically via nilpotent Taylor expansion (No expm needed!)
% 
%         A_bar_sq = A_bar * A_bar; % Nilpotent: A_bar^3 is exactly 0
% 
%         % Compute the exact discrete rotation increment over dt
%         R_delta = ALLFUNCS.Rexp(-w_B * dt);
%         Block_R_delta = blkdiag(R_delta, R_delta, R_delta, R_delta, R_delta);
% 
%         % Exact state transition matrix Phi (Eq 29a from provided image)
%         Exp_A_bar = eye(15) + A_bar * dt + A_bar_sq * (dt^2 / 2);
%         Phi = Exp_A_bar * Block_R_delta;
% 
%         % Exact input matrix Gamma (Eq 29b from provided image)
%         E_15 = eye(15) * dt + A_bar * (dt^2 / 2) + A_bar_sq * (dt^3 / 6);
%         Gamma = E_15 * Block_R_delta * B_matrix;
% 
%         % Predict the state exactly
%         x_pred = Phi * x + Gamma * a_B;
% 
%         % Predict the covariance
%         % Approximated as Qc * dt to avoid exact covariance integrals for performance
%         Q_d = params.V_noise * dt; 
%         P_cov_pred = Phi * P_cov * Phi' + Q_d;
%         P_cov_pred = 0.5 * (P_cov_pred + P_cov_pred'); % Ensure symmetry
% 
%         % 2. DISCRETE UPDATE (Runs ONLY when a measurement hits)
%         if ~isempty(C_dyn)
% 
%             % Standard Discrete Kalman Gain
%             S = C_dyn * P_cov_pred * C_dyn' + Q_dyn;
%             K = P_cov_pred * C_dyn' / S;
% 
%             % Discrete State Update (Instant jump)
%             x = x_pred + K * (y_bar_dyn - C_dyn * x_pred);
% 
%             % Discrete Covariance Update (Joseph form for numerical stability)
%             I_KC = eye(15) - K * C_dyn;
%             P_cov = I_KC * P_cov_pred * I_KC' + K * Q_dyn * K';
%             P_cov = 0.5 * (P_cov + P_cov'); 
% 
%         else
%             % No measurements: lock in the predictions
%             x = x_pred;
%             P_cov = P_cov_pred;
%         end
% 
%         % Extract Auxiliary States
%         e_B1 = x(7:9);
%         e_B2 = x(10:12);
%         e_B3 = x(13:15);
% 
%         % --- D. ATTITUDE RECONSTRUCTION (NCF) ---
%         sigma = cross(e_B1, current_R' * [1; 0; 0]) + ...
%                 cross(e_B2, current_R' * [0; 1; 0]) + ...
%                 cross(e_B3, current_R' * [0; 0; 1]);
% 
%         w_total = w_B + params.k_att * sigma;
%         current_R = current_R * ALLFUNCS.Rexp(w_total * dt);
% 
%         % Ensure rotation matrix remains proper SO(3)
%         current_R = ALLFUNCS.orthogonalize(current_R);
% 
%         % --- E. STORE ESTIMATES ---
%         estimates.P(:, i) = current_R * x(1:3);
%         estimates.V(:, i) = current_R * x(4:6);
%         estimates.R{i}    = current_R;
%     end
% end
