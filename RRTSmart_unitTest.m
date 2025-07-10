clear all; clc;

% Set up the scene
scene_id = 14;
plotting = 1;
environment = createScene(scene_id);

% Create an instance of RRTStarSmart
rrt_smart = RRTSmart(environment);

% Run the RRT*Smart algorithm
goal_reached = rrt_smart.solve(plotting);

