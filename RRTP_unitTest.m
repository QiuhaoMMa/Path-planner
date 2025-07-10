clear all;clc;close all

scene_id=13;
plotting=1;
environment=createScene(scene_id);
rrt= RRTP(environment,'memory_allocation',10,'occupancy_pdf_resolution',1,'steering_resolution',0.1);
goal_reached=rrt.solve(plotting);

%%
clear all; clc; close all;

scene_ids = [1, 2, 3, 4, 5, 6, 7, 8];  % List of 8 scene IDs
plotting = 0;   
num_iterations = 100;


for s = 1:length(scene_ids)
    scene_id = scene_ids(s);
    
    times = zeros(num_iterations, 1); 
    path_lengths = zeros(num_iterations, 1); 
    
    for i = 1:num_iterations
        environment = createScene(scene_id);
        rrtp = RRTP(environment, 'memory_allocation', 10, 'occupancy_pdf_resolution', 1, 'steering_resolution', 0.1);

        tic;
        goal_reached = rrtp.solve(plotting);
        times(i) = toc;

        if goal_reached
            path_lengths(i) = rrtp.getPathLength(); 
        else
            path_lengths(i) = NaN;
        end

        fprintf('Scene %d - Iteration %d completed. Time: %.2f seconds, Path Length: %.2f\n', scene_id, i, times(i), path_lengths(i));
    end

    average_time = mean(times);
    std_time = std(times);

    average_path_length = mean(path_lengths, 'omitnan');
    std_path_length = std(path_lengths, 'omitnan');

    fprintf('Scene %d - Average Time: %.2f seconds\n', scene_id, average_time);
    fprintf('Scene %d - Average Path Length: %.2f units\n', scene_id, average_path_length);

    mysim.rrtp(s).time = times;
    mysim.rrtp(s).pathlength = path_lengths;
    mysim.rrtp(s).avgtime = average_time;
    mysim.rrtp(s).avgpathlength = average_path_length;
    mysim.rrtp(s).stdtime = std_time;
    mysim.rrtp(s).stdpathlength = std_path_length;
end

save('mysim_rrtp.mat', 'mysim');
