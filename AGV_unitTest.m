clc
clear
% close all

scene_id = 1;
plotting = 1;
environment = createScene(scene_id, false); 
rrt = RRT(environment);

goal_reached = rrt.solve(plotting);


path_indices = rrt.reconstructPath(); 
positions = vertcat(rrt.nodes(path_indices).position);  
disp(positions);                                            

maps = {positions};


figure
environment = createScene(scene_id, false); 
rrt = RRT(environment);
hold on
simulatePathFollowing(maps{1});


