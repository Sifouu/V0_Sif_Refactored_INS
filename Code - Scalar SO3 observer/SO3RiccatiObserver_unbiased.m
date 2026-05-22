classdef SO3RiccatiObserver_unbiased < handle
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
        function obj = SO3RiccatiObserver_unbiased(R_hat, dt)
            obj.dt = dt;
            
            % Initialize estimates
            obj.R_hat = R_hat; 
            
            % Initialize Riccati matrix
            obj.P = diag([0.5 * ones(1,3)]);
            
            obj.S = 0.005 * eye(3,3);
            
        end
        
        function [R_hat] = update(obj, omega_meas, g_ref, m_ref, g_meas, m_meas, g_axes, m_axes)
            % Prediction step
            obj.predict(omega_meas);
            
            % Correction step with scalar measurements
            obj.correct(g_ref, m_ref, g_meas, m_meas, g_axes, m_axes);
            
            R_hat = obj.R_hat;
            bias_hat = obj.bias_hat;
        end
        
        function predict(obj, omega_meas)
            
            A = [zeros(3,3)];
             
            F = eye(3);
            
            % Predicted angular velocity (corrected for bias)
            omega_corrected = omega_meas;

            % Propagate attitude estimate
            obj.R_hat = obj.R_hat * expm(skew(omega_corrected) * obj.dt);
            
            % Propagate Riccati matrix
            obj.P = F * obj.P * F' + obj.S ; %* obj.dt;
        end
        
        function correct(obj, b_1, b_2, b_meas_1, b_meas_2, b_1_axes, b_2_axes)      
        
            basis = [1 0 0; 
                     0 1 0; 
                     0 0 1]; 
        
            y = [];
            C = [];
            
%            b_1_axes = [1];       
%            b_2_axes = [1]; 

                    
            for i = 1:length(b_1_axes)
                axis_idx = b_1_axes(i);
                e_axis = basis(axis_idx,:)';

                y = [y; -e_axis' * b_meas_1 + e_axis' * obj.R_hat'*b_1];
                C = [C; e_axis' * obj.R_hat' * skew(b_1)];
            end
            
            for i = 1:length(b_2_axes)
                axis_idx = b_2_axes(i);
                e_axis = basis(axis_idx,:)';

                y = [y; -e_axis' * b_meas_2 + e_axis' * obj.R_hat'*b_2];
                C = [C; e_axis' * obj.R_hat' * skew(b_2)];
            end

            
            obj.Q = 20.0 * eye(length(y),length(y));
             
            % Riccati gain and matrix update
            obj.K = obj.P * C' / (C * obj.P * C' + obj.Q);
            obj.P = (eye(3) - obj.K * C) * obj.P;
            
            % Compute innovations
            Delta = obj.K * y;
            R_innov = Delta;
            
            % Apply corrections
%            obj.R_hat = expm(- skew(R_innov)) * obj.R_hat; %obj.R_hat * 
            obj.R_hat = expm(- skew(R_innov)) *  obj.R_hat;
        end
    end
end

