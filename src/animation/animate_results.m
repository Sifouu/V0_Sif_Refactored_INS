function animate_results(ground_truth, estimates, errors, filename)
% ANIMATE_RESULTS Creates an MP4 animation of the trajectory and attitude.
%
% Inputs:
%   ground_truth - Struct with .P, .R, .time
%   estimates    - Struct with .P, .R, .time
%   errors       - Struct with .pos_geometric, .vel_geometric, .att_trace
%   filename     - String, output file name (e.g., 'animation.mp4')

    disp('==================================================================');
    disp('  Animation of Scenario (Trajectory + Attitude + Errors)');
    disp('==================================================================');

    % Align time scales and downsample to roughly 30 FPS for animation
    % The estimates array defines the time basis
    time_est = estimates.time;
    N_total = length(time_est);
    n_frames = 300;
    step = max(1, floor(N_total / n_frames));
    indices = 1:step:N_total;
    
    time_anim = time_est(indices);
    P_est_anim = estimates.P(:, indices);
    R_est_anim = estimates.R(indices);
    
    pos_err_anim = errors.pos_geometric(indices);
    vel_err_anim = errors.vel_geometric(indices);
    att_err_anim = errors.att_trace(indices);
    
    % Match ground truth to these timestamps.
    % We assume ground_truth has time array that is either time_gt or same as time.
    % Wait, the ground_truth might not have a .time field explicitly if it wasn't passed,
    % but we should assume it does or we construct it. Let's assume we pass the downsampled ground truth.
    % Since in test_generic_observer.m we created gt_downsampled, we can just use that directly!
    % If it doesn't have time, we can assume it matches time_est.
    P_true_anim = ground_truth.P(:, indices);
    R_true_anim = ground_truth.R(indices);

    %% Setup Figure
    % Use high quality figure properties
    fig = figure('Name', 'Animation', 'Color', 'w', 'Position', [50, 50, 1200, 800]);
    fig.GraphicsSmoothing = 'on';
    
    % --- Row 1, Col 1: 3D Trajectory ---
    ax_traj = subplot(2, 2, 1);
    hold(ax_traj, 'on'); grid(ax_traj, 'on'); view(ax_traj, 3);
    title(ax_traj, '3D Trajectory (True vs Est)');
    xlabel(ax_traj, 'X [m]'); ylabel(ax_traj, 'Y [m]'); zlabel(ax_traj, 'Z [m]');
    
    % Static limits
    all_p = [P_true_anim, P_est_anim];
    min_xyz = min(all_p, [], 2);
    max_xyz = max(all_p, [], 2);
    mid_xyz = (min_xyz + max_xyz) / 2;
    max_range = max(max_xyz - min_xyz);
    
    xlim(ax_traj, [mid_xyz(1) - max_range/2, mid_xyz(1) + max_range/2]);
    ylim(ax_traj, [mid_xyz(2) - max_range/2, mid_xyz(2) + max_range/2]);
    zlim(ax_traj, [mid_xyz(3) - max_range/2, mid_xyz(3) + max_range/2]);
    
    line_true = plot3(ax_traj, nan, nan, nan, 'k-', 'LineWidth', 1.5, 'DisplayName', 'True');
    line_est  = plot3(ax_traj, nan, nan, nan, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Est');
    
    % Generate shapes for the trajectory (scaled up by 3x again as requested)
    [verts_pyr_traj, faces_pyr_traj] = generate_pyramid(1.5);
    [verts_cone_traj, faces_cone_traj] = generate_cone(1.5);
    
    patch_traj_true = patch(ax_traj, 'Vertices', verts_pyr_traj, 'Faces', faces_pyr_traj, ...
                       'FaceColor', 'blue', 'FaceAlpha', 0.8, 'EdgeColor', 'k');
    patch_traj_est  = patch(ax_traj, 'Vertices', verts_cone_traj, 'Faces', faces_cone_traj, ...
                       'FaceColor', 'red', 'FaceAlpha', 0.8, 'EdgeColor', 'k');
                       
    legend(ax_traj, [line_true, line_est], 'Location', 'best');
    
    % --- Row 1, Col 2: Attitude View ---
    ax_att = subplot(2, 2, 2);
    hold(ax_att, 'on'); grid(ax_att, 'on'); view(ax_att, 3);
    title(ax_att, 'Attitude (True vs Est)');
    xlabel(ax_att, 'X'); ylabel(ax_att, 'Y'); zlabel(ax_att, 'Z');
    xlim(ax_att, [-1.5 1.5]); ylim(ax_att, [-1.5 1.5]); zlim(ax_att, [-1.5 1.5]);
    
    % Generate Pyramids and Cones for Attitude Viewer (fixed scale)
    [verts_true, faces_true] = generate_pyramid(0.5);
    [verts_est, faces_est] = generate_cone(0.5);
    
    % Create patch objects
    patch_true = patch(ax_att, 'Vertices', verts_true, 'Faces', faces_true, ...
                       'FaceColor', 'blue', 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    patch_est  = patch(ax_att, 'Vertices', verts_est, 'Faces', faces_est, ...
                       'FaceColor', 'red', 'FaceAlpha', 0.8, 'EdgeColor', 'k');
                   
    % --- Row 2: Error Subplots ---
    ax_pos = subplot(2, 3, 4); hold(ax_pos, 'on'); grid(ax_pos, 'on');
    title(ax_pos, 'Position Error'); xlabel(ax_pos, 'Time [s]'); ylabel(ax_pos, 'Error [m]');
    xlim(ax_pos, [time_anim(1), time_anim(end)]); ylim(ax_pos, [0, max(pos_err_anim)*1.1+1e-5]);
    line_err_pos = plot(ax_pos, nan, nan, 'b-', 'LineWidth', 1.5);
    
    ax_vel = subplot(2, 3, 5); hold(ax_vel, 'on'); grid(ax_vel, 'on');
    title(ax_vel, 'Velocity Error'); xlabel(ax_vel, 'Time [s]'); ylabel(ax_vel, 'Error [m/s]');
    xlim(ax_vel, [time_anim(1), time_anim(end)]); ylim(ax_vel, [0, max(vel_err_anim)*1.1+1e-5]);
    line_err_vel = plot(ax_vel, nan, nan, 'b-', 'LineWidth', 1.5);
    
    ax_att_err = subplot(2, 3, 6); hold(ax_att_err, 'on'); grid(ax_att_err, 'on');
    title(ax_att_err, 'Attitude Error'); xlabel(ax_att_err, 'Time [s]'); ylabel(ax_att_err, 'Trace');
    xlim(ax_att_err, [time_anim(1), time_anim(end)]); ylim(ax_att_err, [0, max(att_err_anim)*1.1+1e-5]);
    line_err_att = plot(ax_att_err, nan, nan, 'b-', 'LineWidth', 1.5);

    disp(['Writing video to: ' filename]);
    v = VideoWriter(filename, 'Motion JPEG AVI');
    v.FrameRate = 30;
    v.Quality = 100; % Maximize JPEG compression quality
    open(v);
    
    for i = 1:length(time_anim)
        t_current = time_anim(i);
        t_target = t_current - 8.0;
        
        % Find start frame for 8-second trail
        start_frame = 1;
        for f = i:-1:1
            if time_anim(f) < t_target
                start_frame = f + 1;
                break;
            end
        end
        
        % Update Trajectory Lines (8-second trail)
        set(line_true, 'XData', P_true_anim(1, start_frame:i), 'YData', P_true_anim(2, start_frame:i), 'ZData', P_true_anim(3, start_frame:i));
        set(line_est,  'XData', P_est_anim(1, start_frame:i),  'YData', P_est_anim(2, start_frame:i),  'ZData', P_est_anim(3, start_frame:i));
        
        % Update Trajectory Shapes (Velocity-aligned)
        pos_t = P_true_anim(:, i);
        vel_t = ground_truth.V(:, indices(i)); % Get true velocity
        R_vel_t = get_rotation_from_velocity(vel_t);
        verts_traj_t = (R_vel_t * verts_pyr_traj')' + pos_t';
        set(patch_traj_true, 'Vertices', verts_traj_t);
        
        pos_e = P_est_anim(:, i);
        vel_e = estimates.V(:, indices(i)); % Get est velocity
        R_vel_e = get_rotation_from_velocity(vel_e);
        verts_traj_e = (R_vel_e * verts_cone_traj')' + pos_e';
        set(patch_traj_est, 'Vertices', verts_traj_e);
        
        % Update Attitude Viewer Shapes
        R_t = R_true_anim{i};
        R_e = R_est_anim{i};
        
        verts_t = (R_t * verts_true')';
        verts_e = (R_e * verts_est')';
        
        set(patch_true, 'Vertices', verts_t);
        set(patch_est, 'Vertices', verts_e);
        
        % Update Errors
        set(line_err_pos, 'XData', time_anim(1:i), 'YData', pos_err_anim(1:i));
        set(line_err_vel, 'XData', time_anim(1:i), 'YData', vel_err_anim(1:i));
        set(line_err_att, 'XData', time_anim(1:i), 'YData', att_err_anim(1:i));
        
        drawnow;
        frame = getframe(fig);
        writeVideo(v, frame);
    end
    
    close(v);
    disp('Animation saved successfully.');
end

function [vertices, faces] = generate_pyramid(scale)
    if nargin < 1, scale = 0.5; end
    % Generates a square base pyramid pointing along X
    w = 0.5 * scale;
    h = 1.0 * scale;
    
    vertices = [
         h,  0,  0;
         0,  w,  w;
         0, -w,  w;
         0, -w, -w;
         0,  w, -w
    ];
    
    faces = [
        1 2 3 NaN;
        1 3 4 NaN;
        1 4 5 NaN;
        1 5 2 NaN;
        2 3 4 5
    ];
end

function [vertices, faces] = generate_cone(scale)
    if nargin < 1, scale = 0.5; end
    % Generates a cone pointing along X
    n_segments = 20;
    r = 0.3 * scale;
    h = 1.0 * scale;
    
    theta = linspace(0, 2*pi, n_segments+1);
    theta(end) = []; % remove duplicate
    y = r * cos(theta);
    z = r * sin(theta);
    
    vertices = zeros(n_segments+1, 3);
    vertices(1, :) = [h, 0, 0]; % apex
    for i = 1:n_segments
        vertices(i+1, :) = [0, y(i), z(i)];
    end
    
    % Padded faces matrix
    faces = nan(n_segments+1, n_segments);
    
    % Sides
    for i = 1:n_segments-1
        faces(i, 1:3) = [1, i+1, i+2];
    end
    faces(n_segments, 1:3) = [1, n_segments+1, 2];
    
    % Base
    faces(n_segments+1, 1:n_segments) = 2:n_segments+1;
end

function R = get_rotation_from_velocity(v)
    % Calculates a rotation matrix that aligns the X-axis with the velocity vector
    vn = norm(v);
    if vn < 1e-6
        R = eye(3);
        return;
    end
    
    x_axis = v / vn;
    z_world = [0; 0; 1];
    
    y_axis = cross(z_world, x_axis);
    yn = norm(y_axis);
    
    if yn < 1e-6
        % Velocity is purely vertical
        y_axis = [0; 1; 0];
    else
        y_axis = y_axis / yn;
    end
    
    z_axis = cross(x_axis, y_axis);
    R = [x_axis, y_axis, z_axis];
end
