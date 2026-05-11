function fig = plot_paper_results(ground_truth, estimates, errors, observer_names)
% PLOT_PAPER_RESULTS Generates a 2x2 figure matching the paper template.
%
% Inputs:
%   ground_truth   - Struct with .P, .V, .R
%   estimates      - Struct OR Cell Array of structs (for multiple observers)
%   errors         - Struct OR Cell Array of structs
%   observer_names - String OR Cell Array of strings for the legend
%
% Outputs:
%   fig - Handle to the generated figure

    % Convert single structs to cell arrays for unified looping
    if ~iscell(estimates)
        estimates = {estimates};
        errors = {errors};
        observer_names = {observer_names};
    end
    
    num_observers = length(estimates);
    % MATLAB default color palette is great, but let's explicitly grab them
    colors = lines(max(7, num_observers)); 

    % Create the figure
    fig = figure('Name', 'Observer Results', 'Color', 'w', 'Position', [100, 100, 900, 700]);
    
    %% Top-Left: Geometric Position Error
    subplot(2,2,1); hold on; grid on;
    for i = 1:num_observers
        plot(estimates{i}.time, errors{i}.pos_geometric, '-', 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    ylabel('$\| p^I - \tilde{R} \hat{p}^I \|$', 'Interpreter', 'latex', 'FontSize', 14);
    xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 12);
    legend(observer_names, 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'northeast');
    set(gca, 'Box', 'on', 'LineWidth', 0.5, 'TickLabelInterpreter', 'latex');
    
    %% Top-Right: Geometric Velocity Error
    subplot(2,2,2); hold on; grid on;
    for i = 1:num_observers
        plot(estimates{i}.time, errors{i}.vel_geometric, '-', 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    ylabel('$\| v^I - \tilde{R} \hat{v}^I \|$', 'Interpreter', 'latex', 'FontSize', 14);
    xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 12);
    legend(observer_names, 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'northeast');
    set(gca, 'Box', 'on', 'LineWidth', 0.5, 'TickLabelInterpreter', 'latex');
    
    %% Bottom-Left: Attitude Trace Error
    subplot(2,2,3); hold on; grid on;
    for i = 1:num_observers
        plot(estimates{i}.time, errors{i}.att_trace, '-', 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    ylabel('trace($I_3 - R \hat{R}^T$)', 'Interpreter', 'latex', 'FontSize', 14);
    xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 12);
    legend(observer_names, 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'northeast');
    set(gca, 'Box', 'on', 'LineWidth', 0.5, 'TickLabelInterpreter', 'latex');
    
    %% Bottom-Right: 3D Trajectory
    subplot(2,2,4); hold on; grid on;
    % Plot Ground Truth (Thick Yellow Line)
    plot3(ground_truth.P(1,:), ground_truth.P(2,:), ground_truth.P(3,:), 'y-', 'LineWidth', 4, 'DisplayName', 'Ground Truth');
    
    % Plot Estimates
    for i = 1:num_observers
        plot3(estimates{i}.P(1,:), estimates{i}.P(2,:), estimates{i}.P(3,:), '-', 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', observer_names{i});
    end
    
    xlabel('x-axis', 'Interpreter', 'latex', 'FontSize', 12);
    ylabel('y-axis', 'Interpreter', 'latex', 'FontSize', 12);
    zlabel('z-axis', 'Interpreter', 'latex', 'FontSize', 12);
    legend('Location', 'best', 'Interpreter', 'none');
    set(gca, 'Box', 'on', 'LineWidth', 0.5, 'TickLabelInterpreter', 'latex');
    
    % Match the isometric-like view from the template
    view(3);
    
end
