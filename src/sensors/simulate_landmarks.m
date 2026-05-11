function [y_meas, bearing, positions_I, time_cam] = simulate_landmarks(P_true, R_true, time_gt, landmark_params)
% SIMULATE_LANDMARKS Generates noisy camera measurements (relative position and bearing).
% It automatically generates random landmark positions distributed around the
% drone's trajectory.
%
% Inputs:
%   P_true          - 3xN matrix of true inertial positions
%   R_true          - 1xN cell array of true rotation matrices (Inertial to Body)
%   time_gt         - 1xN vector of ground truth timestamps
%   landmark_params - Struct containing Camera/Landmark configuration:
%       .frequency         - Camera sampling frequency [Hz]
%       .num_landmarks     - Number of landmarks to generate around the trajectory
%       .noise_std_pos     - White noise standard deviation for 3D relative position [m]
%       .noise_std_bearing - White noise standard deviation for bearing vector
%
% Outputs:
%   y_meas      - 3xMxL array of measured 3D relative positions in body frame
%   bearing     - 3xMxL array of measured normalized bearing vectors in body frame
%   positions_I - 3xL matrix of the generated inertial positions of the landmarks
%   time_cam    - 1xM vector of Camera timestamps

    L = landmark_params.num_landmarks;
    
    % 1. Generate landmark positions randomly around the trajectory
    margin = 5; % Expand the bounding box by 5 meters on all sides
    bounds_min = min(P_true, [], 2) - margin;
    bounds_max = max(P_true, [], 2) + margin;
    
    positions_I = zeros(3, L);
    for i = 1:3
        % Uniform random distribution across the trajectory bounding box
        positions_I(i, :) = bounds_min(i) + (bounds_max(i) - bounds_min(i)) * rand(1, L);
    end
    
    % 2. Define the downsampled time vector for the Camera
    dt_cam = 1 / landmark_params.frequency;
    time_cam = time_gt(1) : dt_cam : time_gt(end);
    M = length(time_cam);
    
    % Interpolate ground truth position to camera frequency
    P_interp = interp1(time_gt, P_true', time_cam, 'linear')';
    
    % Preallocate outputs
    y_meas = zeros(3, M, L);
    bearing = zeros(3, M, L);
    
    % 3. Simulate Measurements
    for k = 1:M
        % Find the closest rotation matrix index (nearest neighbor)
        [~, idx] = min(abs(time_gt - time_cam(k)));
        R = R_true{idx};
        
        for l = 1:L
            % True relative vector in body frame: R^T * (p_land - p_true)
            y_true = R' * (positions_I(:, l) - P_interp(:, k));
            
            % Stereo/Depth Measurement: Add position noise
            noise_pos = landmark_params.noise_std_pos * randn(3, 1);
            y_meas(:, k, l) = y_true + noise_pos;
            
            % Monocular Measurement: Calculate true bearing
            b_true = y_true / norm(y_true);
            
            % Add bearing noise and re-normalize (so it remains a unit vector)
            noise_bearing = landmark_params.noise_std_bearing * randn(3, 1);
            b_noisy = b_true + noise_bearing;
            bearing(:, k, l) = b_noisy / norm(b_noisy);
        end
    end
end
