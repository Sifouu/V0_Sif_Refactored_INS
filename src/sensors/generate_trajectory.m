function [R_true, P_true, V_true, W_true, A_true] = generate_trajectory(frequency, t_final, type, perturbation)
% GENERATE_TRAJECTORY Generates the ground truth trajectory for the INS.
%
% Inputs:
%   frequency    - Sampling frequency in Hz (e.g., 1000)
%   t_final      - Total simulation time in seconds (e.g., 9)
%   type         - String defining trajectory type (e.g., 'circular', 'figure8')
%   perturbation - Struct defining perturbations/noise (e.g., wind gusts)
%
% Outputs:
%   R_true - 1xN cell array of 3x3 rotation matrices (Inertial to Body)
%   P_true - 3xN matrix of inertial positions [m]
%   V_true - 3xN matrix of inertial velocities [m/s]
%   W_true - 3xN matrix of body angular velocities [rad/s]
%   A_true - 3xN matrix of inertial linear accelerations [m/s^2]

    % Time setup
    dt = 1 / frequency;
    time = 0:dt:t_final;
    N = length(time);

    % Initialize outputs
    R_true = cell(1, N);
    P_true = zeros(3, N);
    V_true = zeros(3, N);
    W_true = zeros(3, N);
    A_true = zeros(3, N);

    % Initial attitude condition matching original scripts
    R_true{1} = expm([0 0 pi/2; 0 0 0; -pi/2 0 0]);

    % Define Trajectory Types
    if strcmp(type, 'circular')
        radius = 5;
        omega = 5; % rad/s
        height = 5;
        
        % Position
        P_true = [radius * cos(omega * time);
                  radius * sin(omega * time);
                  height * ones(1, N)];
        
        % Angular Velocity (constant yaw to follow path, small roll/pitch oscillations)
        W_true(1, :) = 0.1 * sin(1.0 * time);
        W_true(2, :) = 0.1 * cos(1.0 * time);
        W_true(3, :) = omega * ones(1, N);
        
    elseif strcmp(type, 'figure8')
        A = 5; % X amplitude
        B = 5; % Y amplitude
        omega = 5;
        height = 5;
        
        % Position (Lissajous curve)
        P_true = [A * sin(omega * time);
                  B * sin(2 * omega * time);
                  height + 0.5 * sin(omega * time)];
        
        % Angular Velocity (varied excitation)
        W_true(1, :) = 0.2 * sin(0.3 * time);
        W_true(2, :) = 0.2 * cos(0.2 * time);
        W_true(3, :) = 0.5 * sin(0.1 * time);
        
    else
        error('Unknown trajectory type. Please use "circular" or "figure8".');
    end

    % Velocity (Numeric differentiation of Position)
    for i = 2:N
        V_true(:, i) = (P_true(:, i) - P_true(:, i-1)) / dt;
    end
    V_true(:, 1) = V_true(:, 2); % Boundary condition
    
    % Linear Acceleration (Numeric differentiation of Velocity)
    for i = 2:N
        A_true(:, i) = (V_true(:, i) - V_true(:, i-1)) / dt;
    end
    A_true(:, 1) = A_true(:, 2); % Boundary condition

    % Apply Perturbations
    if nargin >= 3 && ~isempty(perturbation)
        if isfield(perturbation, 'pos_noise')
            P_true = P_true + perturbation.pos_noise * randn(3, N);
        end
        if isfield(perturbation, 'w_noise')
            W_true = W_true + perturbation.w_noise * randn(3, N);
        end
    end

    % Integrate Attitude using Rodrigues' Formula (expm)
    for i = 1:N-1
        w_vec = W_true(:, i);
        w_norm = norm(w_vec);
        
        if w_norm > 1e-8
            % Skew-symmetric matrix of normalized w
            w_hat = w_vec / w_norm;
            wx = [0 -w_hat(3) w_hat(2); 
                  w_hat(3) 0 -w_hat(1); 
                 -w_hat(2) w_hat(1) 0];
             
            theta = w_norm * dt;
            % Rodrigues' rotation formula
            R_exp = eye(3) + sin(theta)*wx + (1-cos(theta))*(wx*wx);
        else
            R_exp = eye(3);
        end
        
        R_true{i+1} = R_true{i} * R_exp;
    end
end
