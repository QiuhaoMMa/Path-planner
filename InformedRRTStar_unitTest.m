% clear all; clc;
% 
% scene_id = 8;     
% plotting = 1;       
% environment = createScene(scene_id);
% informed_rrt_star = InformedRRTStar(environment);
% goal_reached = informed_rrt_star.solve(plotting);

%%
clear all; clc;

scene_ids = [1, 2, 3, 4, 5, 6, 7, 8];  
plotting = 0;   
num_iterations = 100   ;

for s = 1:length(scene_ids)
    scene_id = scene_ids(s);
    
    times = zeros(num_iterations, 1); 
    path_lengths = zeros(num_iterations, 1); 
    
    for i = 1:num_iterations
        environment = createScene(scene_id);
        informed_rrt_star = InformedRRTStar(environment);

        tic;
        goal_reached = informed_rrt_star.solve(plotting);
        times(i) = toc;

        if goal_reached
            path_lengths(i) = informed_rrt_star.getPathLength();
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

    mysim.informedrrtstar(s).time = times;
    mysim.informedrrtstar(s).pathlength = path_lengths;
    mysim.informedrrtstar(s).avgtime = average_time;
    mysim.informedrrtstar(s).avgpathlength = average_path_length;
    mysim.informedrrtstar(s).stdtime = std_time;
    mysim.informedrrtstar(s).stdpathlength = std_path_length;
end
save('mysim_informedrrtstar.mat', 'mysim');
