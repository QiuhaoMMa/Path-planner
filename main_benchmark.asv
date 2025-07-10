clear all; clc; close all;

num_test = 100;  % Number of test runs per scene
scene_ids = 1:33;  % Scene IDs to run
% methods = {@RRTB,@RRTP,@RRT, @RRTS};  % RRT methods
% methods_names = {'RRTB', 'RRTP', 'RRT', 'RRTS'};

methods = {@RRT, @RRTS,@InformedRRTStar,@RRTSmart,@LazyRRT};  % RRT methods
methods_names = {'RRT', 'RRTS','InformedRRTStar','RRTSmart','LazyRRT'};


% methods = {@RRT, @RRTS,@InformedRRTStar,@RRTSmart,@RRTP,@RRTConnect,@LazyRRT};  % RRT methods
% methods_names = {'RRT', 'RRTS','InformedRRTStar','RRTSmart','RRTP','RRTConnect','LazyRRT'};

plotting = 0;  % Plot interval (set to 1 to enable plotting)

% Initialize results structure
results = struct();

for method = methods
    method_name = func2str(method{1});  % Convert function handle to string for results storage
    results.(method_name).data = zeros(max(scene_ids), num_test, 3);  % Pre-allocate results [scene, test, metrics]
    
    for scene_id = scene_ids
        for i = 1:num_test
            fprintf('Method: %s \t Scene ID: %d \t Test Run: %d \n', method_name, scene_id, i);
            
            % Create environment based on the scene ID
            environment = createScene(scene_id);
            
            % Optionally plot the environment
            if plotting
                environment.plot;
            end
            
            % Instantiate the RRT object with specified parameters
            rrt = method{1}(environment, 'memory_allocation', 1000, 'occupancy_pdf_resolution', 2, 'steering_resolution', 0.3);
            
            % Solve the RRT problem
            goal_reached = rrt.solve(plotting);
            
            % Store results: [computation_time, path_length, num_iter]
            results.(method_name).data(scene_id, i, :) = [rrt.computation_time, rrt.path_length, rrt.num_iter];
        end
        
        % Calculate mean and standard deviation for the scene_id
        scene_data = squeeze(results.(method_name).data(scene_id, :, :));  % Get data for the current scene_id
        results.(method_name).statistics(:,:,scene_id) = [mean(scene_data, 1);std(scene_data, 0, 1);];
        
    end
end

% Display results
disp(results);

clear rrt
% Create a filename with the current timestamp
timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
filename = ['workspace_' timestamp '.mat'];

% Save all workspace variables
save(filename);
%% table
benchmark_table


%% plotting
benchmark_plot

