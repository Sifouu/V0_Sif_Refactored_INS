classdef SO3RiccatiObserver_scalar < handle
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
        function obj = SO3RiccatiObserver_scalar(R_hat, bias_hat, dt)
            
            obj.dt = dt;
            obj.R_hat = R_hat;
            obj.bias_hat = bias_hat;

            obj.P = diag([0.5 * ones(1,3), 0.5 * ones(1,3)]);
            obj.S = diag([0.005 * ones(1,3), 0.005 * ones(1,3)]);
        end
        
        function [R_hat, bias_hat] = update(obj, omega_meas, g_ref, m_ref, g_meas, m_meas, g_axes, m_axes)
            
            obj.predict(omega_meas);
            
            obj.correct(g_ref, m_ref, g_meas, m_meas, g_axes, m_axes);
            
            R_hat = obj.R_hat;
            bias_hat = obj.bias_hat;
        end
        
        function predict(obj, omega_meas)

            A = [zeros(3,3), -obj.R_hat;
                 zeros(3,3), zeros(3,3)];
            F = eye(6) + A * obj.dt;
            
            omega_corrected = omega_meas - obj.bias_hat;
            
            obj.R_hat = obj.R_hat * expm(skew(omega_corrected) * obj.dt);
            
            obj.P = F * obj.P * F' + obj.S * obj.dt;
        end
        
        function correct(obj, b_1, b_2, b_meas_1, b_meas_2, b_1_axes, b_2_axes)
            
            y = [];
            C = [];
            
            basis = eye(3);
            for i = 1:length(b_1_axes)
                axis_idx = b_1_axes(i);
                e_axis = basis(axis_idx,:)';
                y = [y; -e_axis' * b_meas_1 + e_axis' * obj.R_hat'*b_1];
                C = [C; e_axis' * obj.R_hat' * skew(b_1), zeros(1,3)];
            end
            
            for i = 1:length(b_2_axes)
                axis_idx = b_2_axes(i);
                e_axis = basis(axis_idx,:)';
                y = [y; -e_axis' * b_meas_2 + e_axis' * obj.R_hat'*b_2];
                C = [C; e_axis' * obj.R_hat' * skew(b_2), zeros(1,3)];
            end
            
            obj.Q = 20.0 * eye(length(y),length(y));

            obj.K = obj.P * C' / (C * obj.P * C' + obj.Q);
            obj.P = (eye(6) - obj.K * C) * obj.P;
            
            Delta = obj.K * y;
            R_innov = Delta(1:3);
            b_innov = Delta(4:6);
            
            obj.R_hat = expm(- skew(R_innov)) * obj.R_hat;
            obj.bias_hat = obj.bias_hat - b_innov;
        end
    end
end