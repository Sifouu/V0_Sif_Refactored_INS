classdef SO3Observer < handle
    properties
        dt          
        R_hat       
        bias_hat    
        P           
        S           
        Q           
        K           
        A
        C
    end
    
    methods
        function obj = SO3Observer(R_hat, bias_hat, dt)
            obj.dt = dt;
            
            % Initialize estimates
            obj.R_hat = R_hat; 
            obj.bias_hat = bias_hat; 
            
            % Initialize Riccati matrix
            obj.P = diag([0.05^2 * ones(1,3), 0.1^2 * ones(1,3)]);
            
            obj.S = 0.05 * eye(6,6);
            
        end
        
        function [R_hat, bias_hat] = update(obj, omega_meas, R_true, g_ref, m_ref)
            % Prediction step
            obj.predict(omega_meas);
            
            % Correction step with scalar measurements
            obj.correct(R_true, g_ref, m_ref);
            
            R_hat = obj.R_hat;
            bias_hat = obj.bias_hat;
        end
        
        function predict(obj, omega_meas)
            
            A = [zeros(3,3), -obj.R_hat;
                 zeros(3,3), zeros(3,3)];
             
            F = eye(6) + A * obj.dt;
            
            % Linearized error dynamics matrix
  %          F = [eye(3), - obj.R_hat * obj.dt;
  %               zeros(3,3), eye(3)];
            
            % Predicted angular velocity (corrected for bias)
            omega_corrected = omega_meas - obj.bias_hat;

            % Propagate attitude estimate
            obj.R_hat = obj.R_hat * expm(skew(omega_corrected) * obj.dt);
            
            % Propagate Riccati matrix
            obj.P = F * obj.P * F' + obj.S ; %* obj.dt;
        end
        
        function correct(obj, R_true, b_1, b_2)      
        
            basis = [1 0 0; 
                     0 1 0; 
                     0 0 1]; 
        
            y = [];
            C = [];
            
            b_1_axes = [1];       
            b_2_axes = [1]; 

%             for i = 1:length(b1_axes)
%                 axis_idx = b1_axes(i);
%                 e_axis = basis(axis_idx,:)';
% 
%                 y = [y; -e_axis' * R_true'*b1 + e_axis' * obj.R_hat'*b1];
%                 C = [C; e_axis' * obj.R_hat' * skew(b1), zeros(1,3)];
%             end
%             
%             for i = 1:length(b2_axes)
%                 axis_idx = b2_axes(i);
%                 e_axis = basis(axis_idx,:)';
% 
%                 y = [y; -e_axis' * R_true'*b2 + e_axis' * obj.R_hat'*b2];
%                 C = [C; e_axis' * obj.R_hat' * skew(b2), zeros(1,3)];
%             end
%              
                        
            for i = 1:length(b_1_axes)
                axis_idx = b_1_axes(i);
                e_axis = basis(axis_idx,:)';

                y = [y; -e_axis' * R_true'*b_1 + e_axis' * obj.R_hat'*b_1];
                C = [C; e_axis' * obj.R_hat' * skew(b_1), zeros(1,3)];
            end
            
            for i = 1:length(b_2_axes)
                axis_idx = b_2_axes(i);
                e_axis = basis(axis_idx,:)';

                y = [y; -e_axis' * R_true'*b_2 + e_axis' * obj.R_hat'*b_2];
                C = [C; e_axis' * obj.R_hat' * skew(b_2), zeros(1,3)];
            end

            
            obj.Q = 0.1 * eye(length(y),length(y));
             
            % Riccati gain and matrix update
            obj.K = obj.P * C' / (C * obj.P * C' + obj.Q);
            obj.P = (eye(6) - obj.K * C) * obj.P;
            
            % Compute innovations
            Delta = obj.K * y;
            R_innov = Delta(1:3);
            b_innov = Delta(4:6);
            
            % Apply corrections
%            obj.R_hat = expm(- skew(R_innov)) * obj.R_hat; %obj.R_hat * 
            obj.R_hat = expm(- skew(R_innov)) *  obj.R_hat;
            obj.bias_hat = obj.bias_hat - b_innov;
        end
    end
end

