function [velCmd, distanceToGoal, goalReached] = myMobileRobotController(controller, pose, goalPoint, goalRadius)
% myMobileRobotController
% A unified helper function for computing control commands for mobile robot models
%
% Inputs:
%   controller     - controllerPurePursuit object
%   pose           - current robot pose [x; y; theta]
%   goalPoint      - target position [x; y]
%   goalRadius     - stopping distance threshold
%
% Outputs:
%   velCmd         - [v; omega] control command
%   distanceToGoal - Euclidean distance from robot to goal
%   goalReached    - true if within goalRadius

    distanceToGoal = norm(pose(1:2) - goalPoint(:));
    goalReached = distanceToGoal < goalRadius;

    if goalReached
        v = 0;
        omega = 0;
    else
        [v, omega] = controller(pose);
    end

    velCmd = [v; omega];
end
