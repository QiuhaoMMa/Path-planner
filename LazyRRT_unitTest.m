clear all; clc;


scene_id = 4;
plotting = 1;
environment = createScene(scene_id);


lazy_rrt = LazyRRT(environment);
goal_reached = lazy_rrt.solve(plotting);