clc;
% Assuming `results`, `scene_ids`, `methods_names`, and `methods` are defined.

metrics = {'Computation Time (s)', 'Path Length', 'Iterations'};

% Pre-allocate a cell array for the table
table_data = cell(length(scene_ids) * length(methods_names), 6);  % Scene, Method, MeanTime, MeanPath, MeanIter, StdTime, StdPath, StdIter

% Fill table_data with the statistics
row = 1;
for scene_id = scene_ids
    for method_name = methods_names(1:numel(methods))
        method_data = results.(method_name{1}).statistics(:,:,scene_id);
        table_data{row, 1} = scene_id;  % Scene ID
        table_data{row, 2} = method_name{1};  % Method
        table_data{row, 3} = method_data(1,1);  % Mean Time
        table_data{row, 4} = method_data(1,2);  % Mean Path Length
        table_data{row, 5} = method_data(1,3);  % Mean Iterations
        table_data{row, 6} = method_data(2,1);  % Std Time
        table_data{row, 7} = method_data(2,2);  % Std Path Length
        table_data{row, 8} = method_data(2,3);  % Std Iterations
        row = row + 1;
    end
end

% Convert to a table and display
table_output = cell2table(table_data, 'VariableNames', ...
    {'Scene', 'Method', 'MeanTime(s)', 'MeanPath', 'MeanIterations', 'StdTime(s)', 'StdPath', 'StdIterations'});
disp(table_output);
