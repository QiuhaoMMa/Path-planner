clc;
clear;
close all;

num_test = 10;               % Trials per map
scene_ids_static = 34:39;    % Static maps
scene_ids_dynamic = 100:102; % Dynamic maps
plotting = 0;                % Disable tree plotting

% === Methods to test ===
methods = {@RRT, @RRTS, @InformedRRTStar, @RRTSmart, @LazyRRT};
methods_names = {'RRT', 'RRTS', 'InformedRRTStar', 'RRTSmart', 'LazyRRT'};

% === Initialize results ===
results = struct();
for m = 1:length(methods)
    method_name = methods_names{m};
    % Static: [time, path_length]
    results.(method_name).static.data = zeros(length(scene_ids_static), num_test, 2);
    results.(method_name).static.success = zeros(length(scene_ids_static), num_test);

    % Dynamic: [time, path_length, total_dynamic_steps]
    results.(method_name).dynamic.data = zeros(length(scene_ids_dynamic), num_test, 2);
    results.(method_name).dynamic.success = zeros(length(scene_ids_dynamic), num_test);
end

num_steps_dynamic = 10;   % Dynamic frames per scene
sampleTime = 0.05;        % AGV step time

% === Test each method ===
for m = 1:length(methods)
    method = methods{m};
    method_name = methods_names{m};

    fprintf('\n==== Testing %s ====\n', method_name);

    % Static maps first
    for s = 1:length(scene_ids_static)
        scene_id = scene_ids_static(s);
        for i = 1:num_test
            fprintf('Static | Method: %s | Scene %d | Trial %d\n', method_name, scene_id, i);
            try
                environment = createScene(scene_id, false);
                rrt = method(environment);
                goal_reached = rrt.solve(plotting);

                if goal_reached
                    path_indices = rrt.reconstructPath();
                    positions = vertcat(rrt.nodes(path_indices).position);
                    path_length = sum(vecnorm(diff(positions), 2, 2));
                    results.(method_name).static.data(s, i, :) = [rrt.computation_time, path_length];
                    results.(method_name).static.success(s, i) = 1;
                else
                    results.(method_name).static.data(s, i, :) = [rrt.computation_time, NaN];
                    results.(method_name).static.success(s, i) = 0;
                end

            catch ME
                warning('Static | Method: %s | Scene %d | Trial %d failed: %s', ...
                    method_name, scene_id, i, ME.message);
                results.(method_name).static.data(s, i, :) = [NaN, NaN];
                results.(method_name).static.success(s, i) = 0;
            end
        end
    end

    % Then dynamic maps
    for s = 1:length(scene_ids_dynamic)
        scene_id = scene_ids_dynamic(s);
        for i = 1:num_test
            fprintf('Dynamic | Method: %s | Scene %d | Trial %d\n', method_name, scene_id, i);

            try
                initial_pose = [2, 2];
                current_position = initial_pose;
                total_distance = 0;
                total_time = 0;
                goal_reached_flag = false;

                for step = 0:num_steps_dynamic
                    environment = createScene(scene_id, false, step);
                    environment.start = current_position;

                    % Stop early if close
                    if norm(current_position - environment.goal) < 3
                        goal_reached_flag = true;
                        break;
                    end

                    rrt = method(environment, ...
                    'memory_allocation', 5000, ...
                    'steering_resolution', 0.1);

                    max_retry = 3;
                    goal_reached = false;
                    
                    for retry = 1:max_retry
                        goal_reached = rrt.solve(0);
                        if goal_reached
                            break; % 成功跳出
                        end
                    end


                    if goal_reached
                        path_indices = rrt.reconstructPath();
                        positions = vertcat(rrt.nodes(path_indices).position);
                        [next_position, step_time] = simulatePathFollowing(positions, current_position, sampleTime);
                        step_distance = norm(next_position - current_position);

                        current_position = next_position;
                        total_distance = total_distance + step_distance;
                        total_time = total_time + step_time;

                    else
                        warning('Dynamic | Method: %s | Scene %d | Trial %d failed at step %d.', ...
                            method_name, scene_id, i, step);
                        goal_reached_flag = false;
                        break;
                    end
                end

                if goal_reached_flag
                    results.(method_name).dynamic.data(s, i, :) = [total_time, total_distance];
                    results.(method_name).dynamic.success(s, i) = 1;
                else
                    results.(method_name).dynamic.data(s, i, :) = [NaN, NaN];
                    results.(method_name).dynamic.success(s, i) = 0;
                end

            catch ME
                warning('Dynamic | Method: %s | Scene %d | Trial %d error: %s', ...
                    method_name, scene_id, i, ME.message);
                results.(method_name).dynamic.data(s, i, :) = [NaN, NaN];
                results.(method_name).dynamic.success(s, i) = 0;
            end
        end
    end
end

% === Compute and display summary ===
fprintf('\n====== Summary (Static Maps) ======\n');
layout_names = {'Manufacturing Cell Layout', 'Amazon Warehouse Layout'};
layout_ranges = {[34, 35, 36], [37, 38, 39]};

for m = 1:length(methods)
    method_name = methods_names{m};
    data = results.(method_name).static.data;
    success = results.(method_name).static.success;

    mean_time = mean(data(:, :, 1), 2, 'omitnan');
    std_time = std(data(:, :, 1), 0, 2, 'omitnan');
    mean_length = mean(data(:, :, 2), 2, 'omitnan');
    std_length = std(data(:, :, 2), 0, 2, 'omitnan');
    success_rate = sum(success, 2) / num_test;

    fprintf('\n--- %s ---\n', method_name);
    for l = 1:2
        range = layout_ranges{l};
        fprintf('\n[%s]\n', layout_names{l});
        fprintf('Scene\tSuccess\tTime (mean±std)\tLength (mean±std)\n');
        for scene_id = range
            idx = scene_id - 33;
            fprintf('%2d\t%.2f\t%.2f ± %.2f\t%.2f ± %.2f\n', ...
                scene_id, success_rate(idx), ...
                mean_time(idx), std_time(idx), ...
                mean_length(idx), std_length(idx));
        end
    end
end

fprintf('\n====== Summary (Dynamic Maps) ======\n');
for m = 1:length(methods)
    method_name = methods_names{m};
    data = results.(method_name).dynamic.data;
    success = results.(method_name).dynamic.success;

    mean_time = mean(data(:, :, 1), 2, 'omitnan');
    std_time = std(data(:, :, 1), 0, 2, 'omitnan');
    mean_dist = mean(data(:, :, 2), 2, 'omitnan');
    std_dist = std(data(:, :, 2), 0, 2, 'omitnan');
    success_rate = sum(success, 2) / num_test;

    fprintf('\n--- %s ---\n', method_name);
    fprintf('Scene\tSuccess\tTime (mean±std)\tDistance (mean±std)\n');
    for s = 1:length(scene_ids_dynamic)
        fprintf('%3d\t%.2f\t%.2f ± %.2f\t%.2f ± %.2f\n', ...
            scene_ids_dynamic(s), success_rate(s), ...
            mean_time(s), std_time(s), ...
            mean_dist(s), std_dist(s));
    end
end

% === Save all results ===
save('RRT_StaticAndDynamic_allMethods_results.mat', 'results');

fprintf('\n✅ All static and dynamic tests complete.\n');
fprintf('Results saved to RRT_StaticAndDynamic_allMethods_results.mat\n');
