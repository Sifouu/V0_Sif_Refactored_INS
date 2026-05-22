function estimates = lcss_attitude_observer(measurements, init_state, params)
% LCSS_ATTITUDE_OBSERVER  Riccati-based linear attitude observer (LCSS 2025).
%
% Implements the attitude observer from the LCSS 2025 paper using a
% vectorized rotation-matrix state  x = vec(R^T) ∈ R^9.
% The same function is called once per measurement case; the case is
% fully defined by the C matrices and noise covariances supplied in params.
%
% --------------------------------------------------------------------------
% Inputs
%   measurements  - Struct with sensor data:
%       .imu.time    [1×N]  IMU timestamps (s)
%       .imu.w_B     [3×N]  Gyroscope   (rad/s, body frame)
%       .imu.a_B     [3×N]  Gravity vector in body frame  g_B = −R^T [0;0;g]
%       .mag.time    [1×M]  Magnetometer timestamps (s)
%       .mag.mag_meas [3×M] Magnetometer readings (body frame)
%
%   init_state    - Struct with initial estimates:
%       .R  [3×3]  Initial rotation matrix (SO(3))
%
%   params        - Struct with tuning parameters:
%       .P0          [9×9]  Initial Riccati covariance  (default 0.001*I)
%       .sigma_gyro  [3×1]  Gyro noise std              (rad/s)
%       .sigma_acc   [ka×1] Accel noise std per measured component
%       .sigma_mag   [km×1] Mag   noise std per measured component
%       .acc_axes    [ka×1] Indices of accel axes measured (e.g., [1;2;3] or [3])
%       .mag_axes    [km×1] Indices of mag axes measured
%       .g_ref       [3×1]  Inertial gravity reference (default [0;0;1])
%       .m_ref       [3×1]  Inertial magnetic reference
%       .eps_reg     [1×1]  Regularisation floor (default 1e-10)
%
% --------------------------------------------------------------------------
% Outputs
%   estimates     - Struct with attitude history:
%       .R     {1×N}  Cell array of 3×3 rotation matrices
%       .time  [1×N]  Time vector (= imu.time)
%
% --------------------------------------------------------------------------
% Notes
%   • This is a pure-attitude observer; .P and .V are NOT populated.
%   • The observer state is projected back onto SO(3) at every step via SVD.
%   • To compare cases, call this function once per (C_acc, C_mag) pair and
%     compute  trace(I - R_true{k} * R_hat{k}')  in the calling script.
% --------------------------------------------------------------------------

    %% 1. Unpack / defaults
    time_imu = measurements.imu.time;
    w_imu    = measurements.imu.w_B;   % [3×N]
    g_B_imu  = measurements.imu.a_B;   % [3×N]  gravity in body (name kept for clarity)

    time_mag = measurements.mag.time;
    m_B_mag  = measurements.mag.mag_meas;  % [3×M]

    N  = numel(time_imu);
    dt = time_imu(2) - time_imu(1);       % assume uniform IMU rate

    % Initial covariance
    if isfield(params, 'P0'),       P_cov = params.P0;
    else,                           P_cov = 0.001 * eye(9); end


    % Covariance matrices
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

    ka = length(acc_axes);
    km = length(mag_axes);

    % Dynamically build C matrices
    basis = eye(3);
    C_acc = zeros(ka, 9);
    for i = 1:ka
        axis_idx = acc_axes(i);
        e_axis = basis(axis_idx, :)';
        C_acc(i, :) = -kron(g_ref, e_axis)';
    end

    C_mag = zeros(km, 9);
    if km > 0
        for i = 1:km
            axis_idx = mag_axes(i);
            e_axis = basis(axis_idx, :)';
            C_mag(i, :) = kron(m_ref, e_axis)';
        end
    end

    %% 2. Initialise state
    % x = vec(R^T)  —  columns of R^T stacked
    R_init = ALLFUNCS.orthogonalize(init_state.R);
    x      = reshape(R_init', 9, 1);

    %% 3. Preallocate outputs
    estimates.R    = cell(1, N);
    estimates.time = time_imu;
    estimates.R{1} = R_init;

    i_mag = 1;   % magnetometer sample pointer

    %% 4. Main loop
    for k = 1:N-1
        w_B = w_imu(:, k);

        % ----------------------------------------------------------------
        % A. PROCESS NOISE   M = dt * N * diag(σ_gyro²) * N'  + ε I
        % N = -blkdiag(skew(x_col1), skew(x_col2), skew(x_col3))
        % ----------------------------------------------------------------
        N_mat = -[ALLFUNCS.skew(x(1:3)); ...
                  ALLFUNCS.skew(x(4:6)); ...
                  ALLFUNCS.skew(x(7:9))];                  % [9×3]
        M_k   = dt * N_mat * Q_gyro * N_mat';% + eps_r * eye(9); 

        % ----------------------------------------------------------------
        % B. STATE TRANSITION   A_dis = blkdiag(dR', dR', dR')
        % ----------------------------------------------------------------
        dR    = ALLFUNCS.Rexp(w_B * dt);
        A_dis = blkdiag(dR', dR', dR');

        % ----------------------------------------------------------------
        % C. RICCATI PREDICT
        % ----------------------------------------------------------------
        P_cov = A_dis * P_cov * A_dis' +  M_k ;
        P_cov = 0.5 * (P_cov + P_cov');

        % ----------------------------------------------------------------
        % D. ACCELEROMETER UPDATE  (every IMU step)
        % ----------------------------------------------------------------
        Q_acc = R_acc(acc_axes, acc_axes);

        S_acc = C_acc * P_cov * C_acc' + Q_acc;
        K_acc = P_cov * C_acc' / S_acc;

        % Select only the observed rows of the 3-D gravity measurement
        y_acc     = g_B_imu(acc_axes, k);          % [ka×1]
        innov_acc = y_acc - C_acc * (A_dis * x);   % [ka×1]
        x         = A_dis * x + K_acc * innov_acc;

        P_cov = (eye(9) - K_acc * C_acc) * P_cov;
        P_cov = 0.5 * (P_cov + P_cov');

        % ----------------------------------------------------------------
        % E. MAGNETOMETER UPDATE  (asynchronous — gated by time)
        % ----------------------------------------------------------------
        if ~isempty(C_mag) && ...
           i_mag <= numel(time_mag) && ...
           time_imu(k) >= time_mag(i_mag)

            Q_mag  = R_mag(mag_axes, mag_axes);

            S_mag = C_mag * P_cov * C_mag' + Q_mag;
            K_mag = P_cov * C_mag' / S_mag;

            % Select only the observed rows of the 3-D magnetometer measurement
            y_mag     = m_B_mag(mag_axes, i_mag);   % [km×1]
            innov_mag = y_mag - C_mag * x;
            x         = x + K_mag * innov_mag;

            P_cov = (eye(9) - K_mag * C_mag) * P_cov;
            P_cov = 0.5 * (P_cov + P_cov');

            i_mag = i_mag + 1;
        end

        % ----------------------------------------------------------------
        % F. PROJECT BACK TO SO(3)  via SVD
        % ----------------------------------------------------------------
        R_k = reshape(x, 3, 3)';          % un-vectorize  (R^T)^T = R
        R_k = ALLFUNCS.orthogonalize(R_k);
        x   = reshape(R_k', 9, 1);        % re-vectorize after projection

        estimates.R{k+1} = R_k;


        
    end
end
