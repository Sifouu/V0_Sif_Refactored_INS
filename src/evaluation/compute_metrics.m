function [errors, rmse] = compute_metrics(ground_truth, estimates)
% COMPUTE_METRICS Calculates Euclidean and Geometric errors, plus RMSE.
%
% Inputs:
%   ground_truth - Struct with fields:
%       .P - 3xN matrix of true positions
%       .V - 3xN matrix of true velocities
%       .R - 1xN cell array of true rotation matrices
%   estimates - Struct with fields:
%       .P - 3xN matrix of estimated positions
%       .V - 3xN matrix of estimated velocities
%       .R - 1xN cell array of estimated rotation matrices
%
% Outputs:
%   errors - Struct with time-series error arrays (1xN vectors)
%       .pos_euclidean
%       .pos_geometric
%       .vel_euclidean
%       .vel_geometric
%       .att_trace
%   rmse - Struct with scalar RMSE values
%       .position_euclidean
%       .position_geometric
%       .velocity_euclidean
%       .velocity_geometric
%       .attitude_trace

    N = size(ground_truth.P, 2);
    
    % Preallocate error arrays
    errors.pos_euclidean = zeros(1, N);
    errors.pos_geometric = zeros(1, N);
    errors.vel_euclidean = zeros(1, N);
    errors.vel_geometric = zeros(1, N);
    errors.att_trace     = zeros(1, N);
    
    for i = 1:N
        P_true = ground_truth.P(:, i);
        V_true = ground_truth.V(:, i);
        R_true = ground_truth.R{i};
        
        P_est = estimates.P(:, i);
        V_est = estimates.V(:, i);
        R_est = estimates.R{i};
        
        % Euclidean errors (L2 norm)
        errors.pos_euclidean(i) = norm(P_true - P_est);
        errors.vel_euclidean(i) = norm(V_true - V_est);
        
        % Geometric errors (Right-Invariant formulation)
        errors.pos_geometric(i) = norm(R_true' * P_true - R_est' * P_est);
        errors.vel_geometric(i) = norm(R_true' * V_true - R_est' * V_est);
        
        % Attitude Trace error
        % Note: Using R_true' * R_est. For perfect estimation, this is Identity,
        % and trace is 3. So 0.5 * trace(I - R_true'*R_est) is 0.
        errors.att_trace(i) = 0.5 * trace(eye(3) - R_true' * R_est);
    end
    
    % Compute scalar RMSE values
    rmse.position_euclidean = sqrt(mean(errors.pos_euclidean.^2));
    rmse.position_geometric = sqrt(mean(errors.pos_geometric.^2));
    rmse.velocity_euclidean = sqrt(mean(errors.vel_euclidean.^2));
    rmse.velocity_geometric = sqrt(mean(errors.vel_geometric.^2));
    rmse.attitude_trace     = sqrt(mean(errors.att_trace.^2));
end
