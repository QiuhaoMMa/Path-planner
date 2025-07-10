clear all;clc

scene_id=14;
plotting=1;
environment=createScene(scene_id);
rrt= RRTS(environment);
goal_reached=rrt.solve(plotting);

%%
clear all; clc;

scene_ids = [1, 2, 3, 4, 5, 6, 7, 8];  
plotting = 0;   
num_iterations = 100;

for s = 1:length(scene_ids)
    scene_id = scene_ids(s);
    
    times = zeros(num_iterations, 1);  
    path_lengths = zeros(num_iterations, 1); 
    
    for i = 1:num_iterations
        environment = createScene(scene_id);
        rrts = RRTS(environment);

        tic;
        goal_reached = rrts.solve(plotting);
        times(i) = toc;

        if goal_reached
            path_lengths(i) = rrts.getPathLength(); 
        else
            path_lengths(i) = NaN;
        end

        fprintf('Scene %d - Iteration %d completed. Time: %.2f seconds, Path Length: %.2f\n', scene_id, i, times(i), path_lengths(i));
    end

    average_time = mean(times);
    std_time = std(times);

    average_path_length = mean(path_lengths, 'omitnan');
    std_path_length = std(path_lengths, 'omitnan');

    % fprintf('Scene %d - Average Time: %.2f seconds\n', scene_id, average_time);
    % fprintf('Scene %d - Average Path Length: %.2f units\n', scene_id, average_path_length);

    mysim.rrts(s).time = times;
    mysim.rrts(s).pathlength = path_lengths;
    mysim.rrts(s).avgtime = average_time;
    mysim.rrts(s).avgpathlength = average_path_length;
    mysim.rrts(s).stdtime = std_time;
    mysim.rrts(s).stdpathlength = std_path_length;
end

save('mysim_rrts.mat', 'mysim');
