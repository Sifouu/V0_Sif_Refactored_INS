function [m_B_meas, time_mag] = simulate_mag(R_true, time_gt, mag_params)
% SIMULATE_MAG Generates noisy Magnetometer measurements from ground truth attitude.
%
% Inputs:
%   R_true     - 1xN cell array of true rotation matrices (Inertial to Body)
%   time_gt    - 1xN vector of ground truth timestamps
%   mag_params - Struct containing Magnetometer configuration:
%       .frequency - Magnetometer sampling frequency [Hz]
%       .noise_std - Magnetometer white noise standard deviation
%       .m_I       - 3x1 Inertial magnetic field vector (default: [1/sqrt(2); 0; 1/sqrt(2)])
%
% Outputs:
%   m_B_meas - 3xM matrix of measured magnetic field in body frame
%   time_mag - 1xM vector of Magnetometer timestamps

    % Default inertial magnetic field if not provided
    if ~isfield(mag_params, 'm_I')
        mag_params.m_I = [1/sqrt(2); 0; 1/sqrt(2)];
    end
    
    % Define the downsampled time vector for the Magnetometer
    dt_mag = 1 / mag_params.frequency;
    time_mag = time_gt(1) : dt_mag : time_gt(end);
    M = length(time_mag);
    
    % Preallocate outputs
    m_B_meas = zeros(3, M);
    
    % Generate white noise
    mag_noise = mag_params.noise_std * randn(3, M);
    
    for i = 1:M
        % Find the closest rotation matrix index (nearest neighbor)
        [~, idx] = min(abs(time_gt - time_mag(i)));
        R = R_true{idx};
        
        % Equation: m_meas = R^T * m_I + noise  (No bias as requested)
        m_true_body = R' * mag_params.m_I;
        m_B_meas(:, i) = m_true_body + mag_noise(:, i);
    end
end
