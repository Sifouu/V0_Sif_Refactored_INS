classdef SO3ComplementaryFilter_unbiased < handle
    properties
        dt          
        R_hat        
        k1          % Gravity correction gain
        k2          % Magnetometer correction gain
    end
    
    methods
        function obj = SO3ComplementaryFilter_unbiased(R_hat, dt)
            obj.dt = dt;
            obj.R_hat = R_hat; 
            
            % Complementary filter gains
            obj.k1 = 2.5;   % Gravity correction
            obj.k2 = 2.5;  % Magnetometer correction  
        end
        
        function [R_hat, bias_hat] = update(obj, omega_meas, g_ref, m_ref, g_meas, m_meas) %g_ref, m_ref, g_meas, m_meas)
            % Prediction step
            obj.predict(omega_meas);
            
            % Correction step
            obj.correct(g_ref, m_ref, g_meas, m_meas) %g_ref, m_ref, g_meas, m_meas);
            
            R_hat = obj.R_hat;
        end
        
        function predict(obj, omega_meas)
            % Corrected angular velocity
            omega_corrected = omega_meas;
            
            % Propagate attitude
            obj.R_hat = obj.R_hat * expm(skew(omega_corrected) * obj.dt);
        end
        
        function correct(obj, g_ref, m_ref, g_meas, m_meas) %, g_meas, m_meas)
            
            % Normalize measurements
            g_meas = g_meas / norm(g_meas);
            m_meas = m_meas / norm(m_meas);
            g_ref = g_ref / norm(g_ref);
            m_ref = m_ref / norm(m_ref);
            
%            g_meas = R_true' * g_ref;
%            m_meas = R_true' * m_ref;
            
%            skew(g_ref)*m_ref
            
%            skew(g_meas)*m_meas
            
            % Compute innovation vectors
            innov_g = skew(obj.R_hat' * g_ref) * g_meas; %skew(g_ref) * obj.R_hat * g_meas;
            innov_m = skew(obj.R_hat' * m_ref) * m_meas; %skew(m_ref) * obj.R_hat * m_meas;
            
            % Total attitude innovation
            innov_R = obj.k1 * innov_g + obj.k2 * innov_m;
            
            % Apply attitude correction
            obj.R_hat =  obj.R_hat * expm(- skew( innov_R * obj.dt)) ;
            
        end
    end
end