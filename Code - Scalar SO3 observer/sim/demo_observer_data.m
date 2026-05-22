clear; clc; close all;
    
% Read IMU data
[gyr_x,gyr_y,gyr_z] = textread('angular_vel.txt','','delimiter',' ','emptyvalue', NaN);
[acc_x,acc_y,acc_z] = textread('accel.txt','','delimiter',' ','emptyvalue', NaN);
[mag_x,mag_y,mag_z] = textread('mag.txt','','delimiter',' ','emptyvalue', NaN);

dt = 0.01; 
T = 30;  
t = 0:dt:T;
N = length(t);

R_hat = eye(3);
bias_hat = [0; 0; 0];  

% Initialize observer
observer = SO3Observer_data(R_hat, bias_hat, dt);


% Reference measurements
g_ref = [0; 0; -9.81];
g_ref = g_ref/norm(g_ref);
m_ref = [23.724; 1.262; 40.605];
m_ref = m_ref/norm(m_ref);

% Logging results
R_hat_log = zeros(3,3,N);
bias_hat_log = zeros(3,N);
%error_R = zeros(1,N);
%bias_error = zeros(3,N);

for k = 1:N 
    
    omega_meas = [0.0; 0.0; 0]; %[gyr_x(k); gyr_y(k); gyr_z(k)];
    
    
%    g_meas = [acc_x(k); acc_y(k); acc_z(k)];
    g_meas = [acc_x(1); acc_y(1); acc_z(1)];
    g_meas = g_meas/norm(g_meas);
    
%    m_meas = [mag_x(k); mag_y(k); mag_z(k)];
    m_meas = [mag_x(1); mag_y(1); mag_z(1)];
    m_meas = m_meas/norm(g_meas);
    
    [R_hat, bias_hat] = observer.update(omega_meas, g_ref, m_ref, g_meas, m_meas);

    R_hat_log(:,:,k) = R_hat;
    bias_hat_log(:,k) = bias_hat;
        
    
end

%subplot(2,1,2);
plot(t, bias_hat_log(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t, bias_hat_log(2,:), 'g-', 'LineWidth', 1.5);
plot(t, bias_hat_log(3,:), 'm-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Gyro Bias Error');
title('Bias Estimation Error'); legend('X', 'Y', 'Z'); grid on;
