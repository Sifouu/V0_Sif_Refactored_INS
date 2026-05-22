function est = mekf_scalar_attitude_observer(measurements, init_state, params)
% MEKF_SCALAR_ATTITUDE_OBSERVER  Lie-group Multiplicative EKF on SO(3).
%
% Implements an Error-State Kalman Filter where the state is natively
% maintained as a 3x3 rotation matrix, and the covariance is tracked
% in the 3x3 Lie Algebra tangent space.
%
% --------------------------------------------------------------------------
% Inputs
%   measurements  - Struct with sensor data:
%       .imu.time    [1×N]  IMU timestamps (s)
%       .imu.w_B     [3×N]  Gyroscope   (rad/s, body frame)
%       .imu.a_B     [3×N]  Raw accelerometer specific force
%       .mag.time    [1×M]  Magnetometer timestamps (s)
%       .mag.mag_meas [3×M] Magnetometer readings (body frame)
%
%   init_state    - Struct with initial estimates:
%       .R  [3×3]  Initial rotation matrix (SO(3))
%
%   params        - Struct with tuning parameters:
%       .P0          [3×3]  Initial Riccati covariance  (default 0.5*I)
%       .S           [3×3]  Process noise covariance    (default 0.005*I)
%       .sigma_acc   [ka×1] Accel noise std per measured component
%       .sigma_mag   [km×1] Mag   noise std per measured component
%       .acc_axes    [ka×1] Indices of accel axes measured (e.g., [1;2;3] or [3])
%       .mag_axes    [km×1] Indices of mag axes measured
%       .g_ref       [3×1]  Inertial gravity reference (default [0;0;1])
%       .m_ref       [3×1]  Inertial magnetic reference
%
% --------------------------------------------------------------------------
% Outputs
%   est           - Struct with attitude history:
%       .R     {1×N}  Cell array of 3×3 rotation matrices
%       .time  [1×N]  Time vector (= imu.time)
%
% --------------------------------------------------------------------------

    %% 1. Unpack / defaults
    time_imu = measurements.imu.time;
    w_imu    = measurements.imu.w_B;   % [3×N]
    a_imu    = measurements.imu.a_B;   % [3×N] 

    time_mag = measurements.mag.time;
    m_mag    = measurements.mag.mag_meas;  % [3×M]

    N  = numel(time_imu);
    dt = time_imu(2) - time_imu(1);       % assume uniform IMU rate

    % Initial covariance and Process noise
    if isfield(params, 'P0'), P_cov = params.P0;
    else,                     P_cov = 0.5 * eye(3); end

    % Measurement covariance matrices
    Q_gyro = params.Q_gyro;
    R_acc  = params.R_acc;
    R_mag  = params.R_mag;

    % Measurement axes and references
    if isfield(params, 'g_ref'),    g_ref = params.g_ref(:);
    else,                           g_ref = [0;0;1]; end
    if isfield(params, 'm_ref'),    m_ref = params.m_ref(:);
    else,                           m_ref = [1/sqrt(2); 0; 1/sqrt(2)]; end

    if isfield(params, 'acc_axes'), acc_axes = params.acc_axes(:);
    else,                           acc_axes = [1;2;3]; end
    if isfield(params, 'mag_axes'), mag_axes = params.mag_axes(:);
    else,                           mag_axes = []; end

    %% 2. Initialise state
    R_hat = ALLFUNCS.orthogonalize(init_state.R);
    
    basis = eye(3);

    %% 3. Preallocate outputs
    est.R    = cell(1, N);
    est.time = time_imu;
    est.R{1} = R_hat;

    i_mag = 1;   % magnetometer sample pointer

    %% 4. Main loop
    for k = 1:N-1
        omega_meas = w_imu(:, k);

        % ----------------------------------------------------------------
        % A. PREDICTION STEP
        % ----------------------------------------------------------------
        % The original implementation keeps F = eye(3)
        F = eye(3);
        
        % Propagate attitude natively on SO(3)
        R_hat = R_hat * ALLFUNCS.Rexp(omega_meas * dt);
        
        % Propagate 3x3 error-state Riccati matrix
        P_cov = F * P_cov * F' + R_hat * (Q_gyro * dt) * R_hat';

        % ----------------------------------------------------------------
        % B. MEASUREMENT UPDATE STEP
        % ----------------------------------------------------------------
        y = [];
        C = [];
        Q_diag = [];
        
        % Dynamically stack Accelerometer innovation and Jacobian
        g_meas = a_imu(:, k);
        for i = 1:length(acc_axes)
            axis_idx = acc_axes(i);
            e_axis = basis(axis_idx, :)';
            % Standard error-state innovation for specific force (a_meas ≈ -R^T g_ref)
            y = [y; -e_axis' * g_meas - e_axis' * R_hat' * g_ref];
            C = [C; -e_axis' * R_hat' * ALLFUNCS.skew(g_ref)];
            Q_diag = [Q_diag; R_acc(axis_idx, axis_idx)];
        end
        
        % Dynamically stack Magnetometer innovation and Jacobian
        if ~isempty(mag_axes) && i_mag <= numel(time_mag) && time_imu(k) >= time_mag(i_mag)
            m_meas = m_mag(:, i_mag);
            for i = 1:length(mag_axes)
                axis_idx = mag_axes(i);
                e_axis = basis(axis_idx, :)';
                % Standard error-state innovation
                y = [y; -e_axis' * m_meas + e_axis' * R_hat' * m_ref];
                C = [C; e_axis' * R_hat' * ALLFUNCS.skew(m_ref)];
                Q_diag = [Q_diag; R_mag(axis_idx, axis_idx)];
            end
            
            % Only increment pointer when magnetometer data is available
            % We will do the update combining both acc and mag if they align
            % but must not forget to increment pointer.
        end
        
        % If we have any measurements, perform the MEKF update
        if ~isempty(y)
            Q = diag(Q_diag);
           % if isempty(Q), Q = 20.0 * eye(length(y)); end % Fallback
            
            % Riccati gain
            K = P_cov * C' / (C * P_cov * C' + Q);
            
            % Update Covariance
            P_cov = (eye(3) - K * C) * P_cov;
            P_cov = 0.5 * (P_cov + P_cov'); % Ensure symmetry
            
            % Update State via exponential map (innovation on SO3)
            R_innov = K * y;
            R_hat = ALLFUNCS.Rexp(-R_innov) * R_hat;
        end
        
        % Magnetometer pointer increment happens after we've used it
        if ~isempty(mag_axes) && i_mag <= numel(time_mag) && time_imu(k) >= time_mag(i_mag)
            i_mag = i_mag + 1;
        end

        % Save estimate
        est.R{k+1} = R_hat;
    end
end
