clear all;clc;close all

scene_id=1;
plotting=1;
environment=createScene(scene_id);


rrtA= RRT(environment,'memory_allocation',50);
goal_reached=rrtA.solve(plotting);



rrtB= RRT(environment,'memory_allocation',50);
goal_reached=rrtB.solve(plotting);



%%
clc
rrtC=RRTBase.connectTrees(rrtA,10,100);
rrtA.nodes(10)
rrtC.nodes(10)
