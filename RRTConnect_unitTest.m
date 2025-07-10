clear all;clc

scene_id=11;
plotting=1;
environment=createScene(scene_id);
rrt= RRTConnect(environment);
goal_reached=rrt.solve(plotting);