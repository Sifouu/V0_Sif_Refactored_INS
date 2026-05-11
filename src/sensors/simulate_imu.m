function [a_B_meas, w_B_meas, time_imu] = simulate_imu(A_true, W_true, R_true, time_gt, imu_params)
% SIMULATE_IMU Generates noisy IMU measurements from ground truth data.
%
% Inputs:
%   A_true     - 3xN matrix of true inertial linear accelerations [m/s^2]
%   W_true     - 3xN matrix of true body angular velocities [rad/s]
%   R_true     - 1xN cell array of true rotation matrices (Inertial to Body)
%   time_gt    - 1xN vector of ground truth timestamps
%   imu_params - Struct containing IMU configuration:
%       .frequency        - IMU sampling frequency [Hz]
%       .accel_noise_std  - Accelerometer white noise standard deviation
%       .gyro_noise_std   - Gyroscope white noise standard deviation
%       .accel_bias       - 3x1 constant accelerometer bias
%       .gyro_bias        - 3x1 constant gyroscope bias
%       .g                - Gravity constant (default 9.81)
%
% Outputs:
%   a_B_meas - 3xM matrix of measured specific force in body frame
%   w_B_meas - 3xM matrix of measured angular velocity in body frame
%   time_imu - 1xM vector of IMU timestamps

    if ~isfield(imu_params, 'g')
        imu_params.g = 9.81; 
    end
    
    gravity_vector = [0; 0; imu_params.g]; 
    
    % Define the downsampled time vector for the IMU
    dt_imu = 1 / imu_params.frequency;
    time_imu = time_gt(1) : dt_imu : time_gt(end);
    M = length(time_imu);
    
    % Interpolate ground truth kinematics to IMU frequency
    A_interp = interp1(time_gt, A_true', time_imu, 'linear')';
    W_interp = interp1(time_gt, W_true', time_imu, 'linear')';
    
    % Preallocate outputs
    a_B_meas = zeros(3, M);
    w_B_meas = zeros(3, M);
    
    % Generate white noise
    accel_noise = imu_params.accel_noise_std * randn(3, M);
    gyro_noise  = imu_params.gyro_noise_std  * randn(3, M);
    
    for i = 1:M
        % Find the closest rotation matrix index (nearest neighbor)
        % (Interpolating SO(3) matrices is computationally heavy and unnecessary 
        % if ground truth frequency is high enough)
        [~, idx] = min(abs(time_gt - time_imu(i)));
        R = R_true{idx};
        
        % 1. Accelerometer (Specific Force)
        % Equation: a_meas = R^T * (A_true - g) + bias + noise
        a_true_body = R' * (A_interp(:, i) - gravity_vector);
        a_B_meas(:, i) = a_true_body + imu_params.accel_bias + accel_noise(:, i);
        
        % 2. Gyroscope
        % Equation: w_meas = W_true + bias + noise
        w_B_meas(:, i) = W_interp(:, i) + imu_params.gyro_bias + gyro_noise(:, i);
    end
end
