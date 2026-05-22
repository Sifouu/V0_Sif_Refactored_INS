function [estimates, kp_hist] = lcss_mahony_dynamic_attitude_observer(measurements, init_state, params)
% LCSS_MAHONY_DYNAMIC_ATTITUDE_OBSERVER Riccati-based linear attitude observer 
% with innovation-driven dynamic Mahony coupling.
%
% Implements the attitude observer from the LCSS 2025 paper using a vectorized
% rotation-matrix state. The state is dynamically projected to SO(3) using
% a Mahony Complementary Filter whose proportional gain (k_p) is dynamically 
% scaled based on the magnitude of the Euclidean measurement innovation.
%
% --------------------------------------------------------------------------
% Inputs:
%   measurements - Struct with sensor data:
%        .imu.time     [1×N] IMU timestamps (s)
%        .imu.w_B      [3×N] Gyroscope (rad/s, body frame)
%        .imu.a_B      [3×N] Gravity vector in body frame g_B = −R^T[0; 0; g]
%        .mag.time     [1×M] Magnetometer timestamps (s)
%        .mag.mag_meas [3×M] Magnetometer readings (body frame)
%
%   init_state - Struct with initial estimates:
%        .R            [3×3] Initial rotation matrix (SO(3))
%
%   params - Struct with tuning parameters:
%        .P0           [9×9] Initial Riccati covariance
%        .sigma_gyro   [3×1] Gyro noise std (rad/s)
%        .sigma_acc    [ka×1] Accel noise std 
%        .sigma_mag    [km×1] Mag noise std
%        .acc_axes     [ka×1] Indices of accel axes measured
%        .mag_axes     [km×1] Indices of mag axes measured
%        .g_ref        [3×1] Inertial gravity reference
%        .m_ref        [3×1] Inertial magnetic reference
%        .dynamic_alpha[1×1] Gain scaling factor for the innovation error
%        .k_min        [1×1] Minimum proportional gain floor (steady-state)
%
% Outputs:
%   estimates - Struct with attitude history
%        .R            {1×N} Cell array of 3×3 rotation matrices
%        .time         [1×N] Time vector (= imu.time)
%   kp_hist   - Array [1×N-1] of the dynamic proportional gain over time
% --------------------------------------------------------------------------

    %% 1. Unpack / defaults
    time_imu = measurements.imu.time;
    w_imu    = measurements.imu.w_B;   % [3×N]
    g_B_imu  = measurements.imu.a_B;   % [3×N]  gravity in body

    time_mag = measurements.mag.time;
    m_B_mag  = measurements.mag.mag_meas;  % [3×M]

    N  = numel(time_imu);
    dt = time_imu(2) - time_imu(1);       % assume uniform IMU rate

    % Initial covariance
    if isfield(params, 'P0'),       P_cov = params.P0;
    else,                           P_cov = 0.001 * eye(9); end

    % Dynamic Mahony Tuning parameters
    if isfield(params, 'dynamic_alpha'), alpha = params.dynamic_alpha;
    else,                                alpha = 1.5; end
    if isfield(params, 'k_min'),         k_min = params.k_min;
    else,                                k_min = 10.0; end

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
    R_init = ALLFUNCS.orthogonalize(init_state.R);
    x      = reshape(R_init', 9, 1);

    %% 3. Preallocate outputs
    estimates.R    = cell(1, N);
    estimates.time = time_imu;
    estimates.R{1} = R_init;
    
    current_R = R_init;
    i_mag = 1;   % magnetometer sample pointer
    kp_hist = zeros(1, N-1);

    %% 4. Main loop
    for k = 1:N-1
        innov_acc = zeros(3,1);
        innov_mag = zeros(3,1);
        
        w_B = w_imu(:, k);

        % ----------------------------------------------------------------
        % A. PROCESS NOISE  
        % ----------------------------------------------------------------
        N_mat = -[ALLFUNCS.skew(x(1:3)); ...
                  ALLFUNCS.skew(x(4:6)); ...
                  ALLFUNCS.skew(x(7:9))];                  % [9×3]
       
        % Inflate process noise to break double integration
        M_k   = 2000 * dt * N_mat * Q_gyro * N_mat'; 
        
        % ----------------------------------------------------------------
        % B. STATE TRANSITION   A_dis = blkdiag(dR', dR', dR')
        % ----------------------------------------------------------------
        dR    = ALLFUNCS.Rexp(w_B * dt);
        A_dis = blkdiag(dR', dR', dR');

        % ----------------------------------------------------------------
        % C. RICCATI PREDICT
        % ----------------------------------------------------------------
        P_cov = A_dis * P_cov * A_dis' + M_k;
        P_cov = 0.5 * (P_cov + P_cov');

        % ----------------------------------------------------------------
        % D. ACCELEROMETER UPDATE 
        % ----------------------------------------------------------------
        Q_acc = R_acc(acc_axes, acc_axes);
        S_acc = C_acc * P_cov * C_acc' + Q_acc;
        K_acc = P_cov * C_acc' / S_acc;

        y_acc     = g_B_imu(acc_axes, k);          
        innov_acc_measured = y_acc - C_acc * (A_dis * x); 
        x         = A_dis * x + K_acc * innov_acc_measured;
        innov_acc(acc_axes) = innov_acc_measured; % Pad to 3x1 for magnitude calculation

        P_cov = (eye(9) - K_acc * C_acc) * P_cov;
        P_cov = 0.5 * (P_cov + P_cov');

        % ----------------------------------------------------------------
        % E. MAGNETOMETER UPDATE
        % ----------------------------------------------------------------
        if ~isempty(C_mag) && ...
           i_mag <= numel(time_mag) && ...
           time_imu(k) >= time_mag(i_mag)

            Q_mag  = R_mag(mag_axes, mag_axes);
            S_mag = C_mag * P_cov * C_mag' + Q_mag;
            K_mag = P_cov * C_mag' / S_mag;

            y_mag     = m_B_mag(mag_axes, i_mag);  
            innov_mag_measured = y_mag - C_mag * x;
            x         = x + K_mag * innov_mag_measured;
            innov_mag(mag_axes) = innov_mag_measured; % Pad to 3x1

            P_cov = (eye(9) - K_mag * C_mag) * P_cov;
            P_cov = 0.5 * (P_cov + P_cov');

            i_mag = i_mag + 1;
        end

        % ----------------------------------------------------------------
        % F. DYNAMIC MAHONY GAIN TUNING
        % ----------------------------------------------------------------
        % Innovation Error Magnitude Tuning
        error_mag = norm(innov_acc) + norm(innov_mag);
        k_p = k_min + alpha * error_mag; % Scales gain proportionally to error
        kp_hist(k) = k_p;

        % ----------------------------------------------------------------
        % G. ATTITUDE RECONSTRUCTION (MAHONY / NCF)
        % ----------------------------------------------------------------
        v1 = x(1:3);
        v2 = x(4:6);
        v3 = x(7:9);

        sigma = cross(v1, current_R' * [1; 0; 0]) + ...
                cross(v2, current_R' * [0; 1; 0]) + ...
                cross(v3, current_R' * [0; 0; 1]);

        w_total = w_B + k_p * sigma;
        current_R = current_R * ALLFUNCS.Rexp(w_total * dt) ;

        x   = reshape(current_R', 9, 1);  
        estimates.R{k+1} = current_R;
    end
end
