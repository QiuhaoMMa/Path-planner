close all;

% Colors for each method
colors = parula(length(methods_names));

% Predefine marker styles
marker_styles = {'o', 's', 'd', '^', 'v', 'p', 'h'};

% Define opacity levels
opacities = linspace(1, 0.3, numel(scene_ids));

%% --- First Figure: Path Length vs Computation Time ---
figure;
hold on;
for scene_id = scene_ids
    for method_idx = 1:length(methods_names)
        method_data = results.(methods_names{method_idx}).statistics(:, :, scene_id);
        means = method_data(1, :);
        stds = method_data(2, :);
        
        % Error bar for Path Length vs Computation Time
        errorbar(means(2), means(1), stds(1), 'o', ...
                 'Color', colors(method_idx, :), ...
                 'Marker', marker_styles{mod(method_idx-1, length(marker_styles))+1}, ...
                 'LineWidth', 1.5);
    end
end
ylabel('Computation Time (s)', 'FontSize', 22, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('Path Length', 'FontSize', 22, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
legend(methods_names, 'Location', 'BestOutside', 'FontSize', 22, 'FontName', 'Times New Roman');
grid on;
set(gca, 'YScale', 'log');
set(gca, 'XScale', 'log');
set(gca, 'FontSize', 22); % Set axis numbers' font size
set(gca, 'FontName', 'Times New Roman');
set(gcf, 'Position', [100, 100, 800, 400]); % Adjust figure size
saveas(gcf, 'PathLength_vs_ComputationTime.png'); % Save the first figure

%% --- Second Figure: Iterations vs Computation Time ---
figure;
hold on;
for scene_id = scene_ids
    for method_idx = 1:length(methods_names)
        method_data = results.(methods_names{method_idx}).statistics(:, :, scene_id);
        means = method_data(1, :);
        stds = method_data(2, :);
        
        % Error bar for Iterations vs Computation Time
        errorbar(means(3), means(1), stds(1), 'o', ...
                 'Color', colors(method_idx, :), ...
                 'Marker', marker_styles{mod(method_idx-1, length(marker_styles))+1}, ...
                 'LineWidth', 1.5);
    end
end
ylabel('Computation Time (s)', 'FontSize', 22, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('Number of Iterations', 'FontSize', 22, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
legend(methods_names, 'Location', 'BestOutside', 'FontSize', 22, 'FontName', 'Times New Roman');
grid on;
set(gca, 'YScale', 'log');
set(gca, 'XScale', 'log');
set(gca, 'FontSize', 22); % Set axis numbers' font size
set(gca, 'FontName', 'Times New Roman');
set(gcf, 'Position', [100, 100, 800, 400]); % Adjust figure size
saveas(gcf, 'Iterations_vs_ComputationTime.png'); % Save the second figure
