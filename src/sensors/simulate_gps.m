function [p_gps_meas, v_gps_meas, is_valid, time_gps] = simulate_gps(P_true, V_true, time_gt, gps_params)
% SIMULATE_GPS Generates noisy GPS measurements and handles signal outages.
%
% Inputs:
%   P_true     - 3xN matrix of true inertial positions [m]
%   V_true     - 3xN matrix of true inertial velocities [m/s]
%   time_gt    - 1xN vector of ground truth timestamps
%   gps_params - Struct containing GPS configuration:
%       .frequency     - GPS sampling frequency [Hz]
%       .noise_std_pos - White noise standard deviation for position [m]
%       .noise_std_vel - White noise standard deviation for velocity [m/s]
%       .outage_start  - (Optional) Start time of GPS outage [s]
%       .outage_end    - (Optional) End time of GPS outage [s]
%
% Outputs:
%   p_gps_meas - 3xM matrix of measured inertial position (NaN during outage)
%   v_gps_meas - 3xM matrix of measured inertial velocity (NaN during outage)
%   is_valid   - 1xM logical array (true if GPS is available)
%   time_gps   - 1xM vector of GPS timestamps

    % Define the downsampled time vector for the GPS
    dt_gps = 1 / gps_params.frequency;
    time_gps = time_gt(1) : dt_gps : time_gt(end);
    M = length(time_gps);
    
    % Interpolate ground truth to GPS frequency
    P_interp = interp1(time_gt, P_true', time_gps, 'linear')';
    V_interp = interp1(time_gt, V_true', time_gps, 'linear')';
    
    % Add white noise
    noise_pos = gps_params.noise_std_pos * randn(3, M);
    noise_vel = gps_params.noise_std_vel * randn(3, M);
    
    p_gps_meas = P_interp + noise_pos;
    v_gps_meas = V_interp + noise_vel;
    
    % Handle Outages
    is_valid = true(1, M);
    
    if isfield(gps_params, 'outage_start') && isfield(gps_params, 'outage_end')
        outage_idx = (time_gps >= gps_params.outage_start) & (time_gps <= gps_params.outage_end);
        
        % Invalidate the measurements during the outage
        is_valid(outage_idx) = false;
        p_gps_meas(:, outage_idx) = NaN;
        v_gps_meas(:, outage_idx) = NaN;
    end
end
