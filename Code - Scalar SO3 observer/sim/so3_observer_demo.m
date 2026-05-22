
% SO(3) Observer for orientation and angular velocity bias estimation
% Uses linearized rotation error dynamics with Riccati design
% Measurements: scalar acceleration and magnetic field 
% Author: T. Bouazza, 23/08/2025   

clear; clc; close all;
    
dt = 0.01;  
T = 100;    
t = 0:dt:T;
N = length(t);
    
    
% True initial conditions
R_true = eye(3); %eul2rotm([0.7 0.1 0.2]) % initial orientation
bias_true = [0.1; 0.08; -0.13];  % constant angular velocity bias [rad/s]
    
% Observer initial conditions (with some error)
%R_hat = axang2rotm([1 0.0 0 0.4]);  % initial estimate with 0.2 rad error
R_hat = eul2rotm([0.7 0.5 0.8]);
bias_hat = [0; 0; 0];  % initial bias estimate

% Initialize observer
observer = SO3Observer(R_hat, bias_hat, dt);

%R_error = R_hat' * R_true;
%sqrt(trace((eye(3) - R_error)'*(eye(3) - R_error)))
    
% Reference vectors
g_ref = [0; 0; 1]; %[0; 0; -9.81];    % gravity reference (NED frame)
m_ref = [1; 0; 1]; %[0.4; 0; 0.25]; %[0.4; 0; -0.25];  % magnetic field reference (normalized)
%m_ref = m_ref / norm(m_ref);
    
% Noise parameters
gyro_noise_std = 0.000;     % gyroscope noise [rad/s]
accel_noise_std = 0.000;     % accelerometer noise [m/s^2]
mag_noise_std = 0.000;      % magnetometer noise
    
% Storage for results
R_true_log = zeros(3,3,N);
R_hat_log = zeros(3,3,N);
bias_true_log = zeros(3,N);
bias_hat_log = zeros(3,N);
error_R = zeros(1,N);
bias_error = zeros(3,N);
    
for k = 1:N        

        % True angular velocity (time-varying for demonstration)
%         omega_true = [0; cos(0.5*t(k)); sin(0.2*t(k))];
        omega_true = [0; 0; cos(0.5*t(k))];
               
%        omega_true = [0.2*sin(0.5*t(k)); 0; 0];
%        omega_true = [0; 0; 0]; 

        % Propagate true R dynamics
        R_true = R_true * expm(skew(omega_true) * dt);
        

        % Gyro measurement (with bias and noise)
        omega_meas = omega_true + bias_true  + gyro_noise_std * randn(3,1);
        
        % True acceleration and magnetic field in the body frame
        
%        accel_body = R_true' * g_ref;  % assuming no linear acceleration
%        mag_body = R_true' * m_ref;
        
        % Add measurement noise
%        accel_meas = accel_body + accel_noise_std * randn(3,1);
%        mag_meas = mag_body + mag_noise_std * randn(3,1);
        
        % Normalize measurements 
%        accel_meas = accel_meas / norm(accel_meas);
%        mag_meas = mag_meas / norm(mag_meas);
        
        
        % Observer update
        [R_hat, bias_hat] = observer.update(omega_meas, R_true, g_ref, m_ref);
        
        % Store results
        R_true_log(:,:,k) = R_true;
        R_hat_log(:,:,k) = R_hat;
        bias_true_log(:,k) = bias_true;
        bias_hat_log(:,k) = bias_hat;
        
        % Compute rotation error angle
        R_error = R_hat * R_true';
        error_R(k) = sqrt(trace((eye(3) - R_error)'*(eye(3) - R_error)));
        
        bias_error(:,k) = bias_hat - bias_true;
 
end
    
% Plot results
figure('Position', [100 100 1200 800]);
    
subplot(2,1,1);
plot(t, error_R, 'b-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Rotation Error');
title('Orientation Estimation Error'); grid on;    
    
subplot(2,1,2);
plot(t, bias_error(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, bias_error(2,:), 'g-', 'LineWidth', 1.5);
plot(t, bias_error(3,:), 'm-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Gyro Bias Error');
title('Bias Estimation Error'); legend('X', 'Y', 'Z'); grid on;

