function estimates = observer_template(measurements, init_state, params)
% OBSERVER_TEMPLATE A skeleton for implementing INS observers (EKF, InEKF, LPV).
%
% Inputs:
%   measurements - Struct containing sensor data:
%       .imu  (time, a_B, w_B)
%       .gps  (time, p_gps_meas, v_gps_meas, is_valid)
%       .mag  (time, mag_meas)
%       .cam  (time, landmark_meas, bearing, positions_I)
%   init_state   - Struct with initial state estimates:
%       .P    (3x1 initial position)
%       .V    (3x1 initial velocity)
%       .R    (3x3 initial rotation matrix)
%   params       - Struct with filter tuning parameters (Q, R matrices, etc.)
%
% Outputs:
%   estimates - Struct containing the estimated state history:
%       .P    (3xN matrix of estimated positions)
%       .V    (3xN matrix of estimated velocities)
%       .R    (1xN cell array of estimated rotation matrices)
%       .time (1xN vector matching IMU timestamps)

    %% 1. Initialization
    time_imu = measurements.imu.time;
    N = length(time_imu);
    
    % Preallocate output arrays for speed
    estimates.P = zeros(3, N);
    estimates.V = zeros(3, N);
    estimates.R = cell(1, N);
    estimates.time = time_imu;
    
    % Initialize State variables
    current_P = init_state.P;
    current_V = init_state.V;
    current_R = init_state.R;
    
    % Initialize Covariance / Riccati matrix
    % P_cov = params.P0;
    
    % Save initial state
    estimates.P(:, 1) = current_P;
    estimates.V(:, 1) = current_V;
    estimates.R{1}    = current_R;
    
    %% 2. Main Estimation Loop
    for i = 2:N
        % Time step
        dt = time_imu(i) - time_imu(i-1);
        
        % Current IMU readings
        a_B = measurements.imu.a_B(:, i-1);
        w_B = measurements.imu.w_B(:, i-1);
        
        % ==========================================
        % A. PREDICTION STEP (Kinematics)
        % ==========================================
        
        % Propagate State (e.g., Euler integration, Rodrigues formula)
        % current_P = ...
        % current_V = ...
        % current_R = ...
        
        % Propagate Covariance (Compute Jacobians A, B, Riccati equation)
        % P_cov = ...
        
        % ==========================================
        % B. UPDATE STEP (Corrections)
        % ==========================================
        
        % Check for GPS measurement
        % (Logic to check if GPS time matches current IMU time and is_valid is true)
        % if gps_available
        %     Compute Innovation (Y - Y_hat)
        %     Compute Kalman Gain (K)
        %     Update State (current_P, current_V, current_R)
        %     Update Covariance (P_cov)
        % end
        
        % Check for Camera/Landmark measurement
        % if cam_available
        %     Compute Innovation
        %     Update State
        %     Update Covariance
        % end
        
        % Check for Magnetometer measurement
        % if mag_available
        %     Compute Innovation
        %     Update State
        %     Update Covariance
        % end
        
        % ==========================================
        % C. STORE ESTIMATES
        % ==========================================
        estimates.P(:, i) = current_P;
        estimates.V(:, i) = current_V;
        estimates.R{i}    = current_R;
    end
end
