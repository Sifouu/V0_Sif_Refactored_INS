clear; clc; close all;

%% Read IMU data

load(['03_undisturbed_slow_rotation_C.mat']);
start_idx = 14000;
end_idx   = 46000;

opt_quat = opt_quat(start_idx:end_idx,:);
imu_gyr  = imu_gyr(start_idx:end_idx,:);
imu_acc  = imu_acc(start_idx:end_idx,:);
imu_mag  = imu_mag(start_idx:end_idx,:);

dt = 1/286;
T  = 50;
N  = length(imu_mag);
t  = linspace(0, T, N);

%% Initial conditions

R_hat    = eye(3);
R_hat2   = eye(3);
R_hat3   = eye(3);
R_hat4   = eye(3);
R_hat_cf = eye(3);

%% Initialize observers

RiccatiObserverFull  = SO3RiccatiObserver_unbiased(R_hat, dt);
RiccatiObserver2     = SO3RiccatiObserver_unbiased(R_hat2, dt);
RiccatiObserver3     = SO3RiccatiObserver_unbiased(R_hat3, dt);
RiccatiObserver4     = SO3RiccatiObserver_unbiased(R_hat4, dt);
ComplementaryFilter  = SO3ComplementaryFilter_unbiased(R_hat_cf, dt);

%% Reference measurements

g_ref = [0; 0; 9.81];
g_ref = g_ref / norm(g_ref);

m_ref = [-0.3680; 13.6868; -39.2062];
m_ref = m_ref / norm(m_ref);

%% Logging results

R_hat_log    = zeros(3,3,N);
R_hat2_log   = zeros(3,3,N);
R_hat3_log   = zeros(3,3,N);
R_hat4_log   = zeros(3,3,N);
R_hat_cf_log = zeros(3,3,N);

%% Measurement axes

g_axes = [1 2 3];
m_axes = [1 2 3];

g_axes2  = [2];
m_axes2  = [2];

g_axes3 = [2 3];
m_axes3 = [2];

g_axes4 = [2 3];
m_axes4 = [1 2];


%% Main loop

for k = 1:N

    omega_meas = imu_gyr(k,:)';

    g_meas = imu_acc(k,:)';
    g_meas = g_meas / norm(g_meas);

    m_meas = imu_mag(k,:)';
    m_meas = m_meas / norm(m_meas);

    R_hat  = RiccatiObserverFull.update(omega_meas, g_ref, m_ref, g_meas, m_meas, g_axes, m_axes);
    R_hat2 = RiccatiObserver2.update(omega_meas, g_ref, m_ref, g_meas, m_meas, g_axes2,  m_axes2);
    R_hat3 = RiccatiObserver3.update(omega_meas, g_ref, m_ref, g_meas, m_meas, g_axes3, m_axes3);
    R_hat4 = RiccatiObserver4.update(omega_meas, g_ref, m_ref, g_meas, m_meas, g_axes4, m_axes4);
    R_hat_cf = ComplementaryFilter.update(omega_meas, g_ref, m_ref, g_meas, m_meas);

    R_hat_log(:,:,k)    = R_hat;
    R_hat2_log(:,:,k)   = R_hat2;
    R_hat3_log(:,:,k)   = R_hat3;
    R_hat4_log(:,:,k)   = R_hat4;
    R_hat_cf_log(:,:,k) = R_hat_cf;
end

%% Rotation error metrics

R_gt = quat2rotm(opt_quat);

R_err   = zeros(3,3,N);
R_err2   = zeros(3,3,N);
R_err3   = zeros(3,3,N);
R_err4   = zeros(3,3,N);
R_err_cf = zeros(3,3,N);

ang_err   = zeros(N,1);
ang_err2   = zeros(N,1);
ang_err3   = zeros(N,1);
ang_err4   = zeros(N,1);
ang_err_cf = zeros(N,1);

incl_err1 = zeros(N,1);

for k = 1:N
    R_err(:,:,k)    = R_gt(:,:,k)' * R_hat_log(:,:,k);
    R_err2(:,:,k)   = R_gt(:,:,k)' * R_hat2_log(:,:,k);
    R_err3(:,:,k)   = R_gt(:,:,k)' * R_hat3_log(:,:,k);
    R_err4(:,:,k)   = R_gt(:,:,k)' * R_hat4_log(:,:,k);
    R_err_cf(:,:,k) = R_gt(:,:,k)' * R_hat_cf_log(:,:,k);

    ang_err(k)    = acos(max(-1, min(1, (trace(R_err(:,:,k))   - 1) / 2)));
    ang_err2(k)   = acos(max(-1, min(1, (trace(R_err2(:,:,k))   - 1) / 2)));
    ang_err3(k)   = acos(max(-1, min(1, (trace(R_err3(:,:,k))   - 1) / 2)));
    ang_err4(k)   = acos(max(-1, min(1, (trace(R_err4(:,:,k))   - 1) / 2)));
    ang_err_cf(k) = acos(max(-1, min(1, (trace(R_err_cf(:,:,k)) - 1) / 2)));

    incl_err(k) = acos(R_err(3,3,k));
end

rmse    = sqrt(mean(rad2deg(ang_err).^2));
rmse2   = sqrt(mean(rad2deg(ang_err2).^2));
rmse3   = sqrt(mean(rad2deg(ang_err3).^2));
rmse4   = sqrt(mean(rad2deg(ang_err4).^2));
rmse_cf = sqrt(mean(rad2deg(ang_err_cf).^2));

fprintf('=== THETA ERROR === \n');
fprintf('Riccati Observer 2 scalar: %.3f \n', rmse2);
fprintf('Riccati Observer 3 scalar: %.3f \n', rmse3);
fprintf('Riccati Observer 4 scalar: %.3f \n', rmse4);
fprintf('Riccati Observer vector: %.3f \n', rmse);
fprintf('Complementary Filter : %.3f \n', rmse_cf);


%% Euler angles

eul_gt    = quat2eul(opt_quat, 'ZYX');
eul_hat   = rotm2eul(R_hat_log, 'ZYX');
eul2_hat  = rotm2eul(R_hat2_log, 'ZYX');
eul3_hat  = rotm2eul(R_hat3_log, 'ZYX');
eul4_hat  = rotm2eul(R_hat4_log, 'ZYX');
eulcf_hat = rotm2eul(R_hat_cf_log, 'ZYX');

eul_gt    = unwrap(eul_gt);
eul_hat   = unwrap(eul_hat);
eul2_hat  = unwrap(eul2_hat);
eul3_hat  = unwrap(eul3_hat);
eul4_hat  = unwrap(eul4_hat);
eulcf_hat = unwrap(eulcf_hat);

angle_rmse = @(true_ang, est_ang) sqrt(nanmean((mod(true_ang - est_ang + pi, 2*pi) - pi).^2));

observers = {eul2_hat, eul3_hat, eul4_hat, eul_hat, eulcf_hat};
names = {'Riccati Observer 2 scalar', 'Riccati Observer 3 scalar', 'Riccati Observer 4 scalar', 'Riccati Observer Vector', 'ComplementaryFilter'};

fprintf('\n=== EULER ANGLE RMSE ===\n');
for i = 1:5
    roll_rmse  = angle_rmse(eul_gt(:,1), observers{i}(:,1));
    pitch_rmse = angle_rmse(eul_gt(:,2), observers{i}(:,2));
    yaw_rmse   = angle_rmse(eul_gt(:,3), observers{i}(:,3));

    total_rmse = sqrt(nanmean( ...
        (mod(eul_gt(:,1) - observers{i}(:,1) + pi, 2*pi) - pi).^2 + ...
        (mod(eul_gt(:,2) - observers{i}(:,2) + pi, 2*pi) - pi).^2 + ...
        (mod(eul_gt(:,3) - observers{i}(:,3) + pi, 2*pi) - pi).^2 ));

    fprintf('%s:\n', names{i});
    fprintf('  Yaw: %.3f deg | Pitch: %.3f deg | Roll: %.3f deg | Total: %.3f deg\n\n', ...
        rad2deg([roll_rmse, pitch_rmse, yaw_rmse, total_rmse]));
end

eul_gt   = rad2deg(eul_gt);
eul_hat  = rad2deg(eul_hat);
eul2_hat = rad2deg(eul2_hat);
eul3_hat = rad2deg(eul3_hat);
eul4_hat = rad2deg(eul4_hat);
eulcf_hat = rad2deg(eulcf_hat);

% Euler angle plots

eul_err    = abs(eul_gt - eul_hat);
eul_err2   = abs(eul_gt - eul2_hat);
eul_err3   = abs(eul_gt - eul3_hat);
eul_err4   = abs(eul_gt - eul4_hat);
eul_err_cf = abs(eul_gt - eulcf_hat);

figure()
set(gca, 'Position', [-0.1,0.1,1.2,0.85])
set(gca, 'OuterPosition', [0,0,1,1])
plot(t, eul_err(:,3), 'b-', 'LineWidth', 1.5);
hold on; grid on;
plot(t, eul_err2(:,3), 'r-', 'LineWidth', 1.5);
plot(t, eul_err3(:,3), 'g-', 'LineWidth', 1.5);
plot(t, eul_err4(:,3), 'm-', 'LineWidth', 1.5);
plot(t, eul_err_cf(:,3), 'y-', 'LineWidth', 1.5);
ylabel('Roll (deg)', 'Interpreter', 'latex', 'FontSize', 16);
title('Euler Angle Errors', 'Interpreter', 'latex', 'FontSize', 16);
lgd = legend('Vector', '2 scalars', '3 scalars', '4 scalars', 'Complementary', 'Location', 'best');
lgd.FontSize = 14;

figure()
set(gca, 'Position', [-0.1,0.1,1.2,0.85])
set(gca, 'OuterPosition', [0,0,1,1])
plot(t, eul_err(:,2), 'b-', 'LineWidth', 1.5);
hold on; grid on;
plot(t, eul_err2(:,2), 'r-', 'LineWidth', 1.5);
plot(t, eul_err3(:,2), 'g-', 'LineWidth', 1.5);
plot(t, eul_err4(:,2), 'm-', 'LineWidth', 1.5);
plot(t, eul_err_cf(:,2), 'y-', 'LineWidth', 1.5);
ylabel('Pitch (deg)', 'Interpreter', 'latex', 'FontSize', 16);

figure()
set(gca, 'Position', [-0.1,0.1,1.2,0.85])
set(gca, 'OuterPosition', [0,0,1,1])
plot(t, eul_err(:,1), 'b-', 'LineWidth', 1.5);
hold on; grid on;
plot(t, eul_err2(:,1), 'r-', 'LineWidth', 1.5);
plot(t, eul_err3(:,1), 'g-', 'LineWidth', 1.5);
plot(t, eul_err4(:,1), 'm-', 'LineWidth', 1.5);
plot(t, eul_err_cf(:,1), 'y-', 'LineWidth', 1.5);
ylabel('Yaw (deg)', 'Interpreter', 'latex', 'FontSize', 16);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 16);

figure()
set(gca, 'Position', [-0.1,0.1,1.2,0.85])
set(gca, 'OuterPosition', [0,0,1,1])
plot(t, rad2deg(ang_err), 'b-', 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(ang_err2), 'g-', 'LineWidth', 1.5);
plot(t, rad2deg(ang_err3), 'm-', 'LineWidth', 1.5);
plot(t, rad2deg(ang_err4), 'r-', 'LineWidth', 1.5);
plot(t, rad2deg(ang_err_cf), 'y-', 'LineWidth', 1.5);
grid on;
ylabel('$\theta$ (deg)', 'Interpreter', 'latex', 'FontSize', 16);
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 16);
title('Angular Error $\theta$', 'Interpreter', 'latex', 'FontSize', 16);
lgd = legend('Vector', '2 scalars', '3 scalars', '4 scalars', 'Complementary', 'Location', 'best');
lgd.FontSize = 14;
