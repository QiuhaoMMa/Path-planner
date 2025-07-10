clc;
clear;
close all;

% === Unit test: Dynamic Map with Full Visualization ===

scene_id = 100;            % Dynamic map ID: 100 = easy version
num_steps = 10;            % Dynamic frames
sampleTime = 0.05;         % AGV controller sample time
plotting = 0;              % Disable RRT tree plotting

initial_pose = [2, 2];
current_position = initial_pose;

step_fraction = 0.15;      % How far to move per frame
obstacle_speed = 1;        % Obstacle speed per frame (inside createScene)

% === Accumulators ===
final_trajectory = current_position;
total_distance = 0;
total_time = 0;

% === Visualization ===
figure('Color', 'w'); 
axis equal;
xlim([0 30]); ylim([0 30]);
hold on;

for step = 0:num_steps

    % Generate dynamic environment for this step
    environment = createScene(scene_id, false, step);
    environment.start = current_position;

    % Check if AGV is close enough to goal
    if norm(current_position - environment.goal) < 3
        fprintf('✅ AGV reached goal vicinity at step %d.\n', step);
        break;
    end

    % Plan path
    rrt = RRT(environment, ...
        'memory_allocation', 5000, ...
        'steering_resolution', 0.1);

    goal_reached = rrt.solve(plotting);

    clf; 
    environment.plot; 
    hold on;

    if goal_reached
        path_indices = rrt.reconstructPath();
        positions = vertcat(rrt.nodes(path_indices).position);

        % Draw current planned path
        plot(positions(:,1), positions(:,2), 'r-', 'LineWidth', 2);

        % Simulate AGV partial motion
        [next_position, step_time] = simulatePathFollowing(positions, current_position, sampleTime, step_fraction);
        step_distance = norm(next_position - current_position);

        % Update
        plot(next_position(1), next_position(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6);
        current_position = next_position;

        final_trajectory = [final_trajectory; current_position];
        total_distance = total_distance + step_distance;
        total_time = total_time + step_time;

        title(sprintf('Dynamic Map %d | Step %d | AGV @ (%.1f, %.1f)', ...
            scene_id, step, current_position(1), current_position(2)));

    else
        warning('❌ No path found at step %d. Terminating.', step);
        break;
    end

    drawnow;
    pause(0.1);
end

% === Final full trajectory ===
fprintf('\n✅ Dynamic Map %d DONE.\nTotal Distance: %.2f units | Total Time: %.2f s\n', ...
    scene_id, total_distance, total_time);

figure('Color', 'w');
environment = createScene(scene_id, false, step);
environment.plot;
hold on;

simulatePathFollowing(final_trajectory);
title(sprintf('Dynamic Map %d | Full Trajectory Playback', scene_id));
axis equal; xlim([0 30]); ylim([0 30]);

