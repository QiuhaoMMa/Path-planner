function [final_position, total_time] = simulatePathFollowing(waypoints, start_position, sampleTime, step_fraction)
    if nargin < 2 || isempty(start_position)
        start_position = waypoints(1,:);
    end
    if nargin < 3 || isempty(sampleTime)
        sampleTime = 0.05;
    end
    if nargin < 4 || isempty(step_fraction)
        step_fraction = 1.0; % Full playback by default
    end

    tVec = 0:sampleTime:40;
    initPose = [start_position(1); start_position(2); 0];

    diffDrive = differentialDriveKinematics(VehicleInputs="VehicleSpeedHeadingRate");
    diffDrive.WheelSpeedRange = [-10 10]*2*pi;

    controller = controllerPurePursuit(Waypoints=waypoints, ...
        DesiredLinearVelocity=3, MaxAngularVelocity=3*pi);

    goalPoint = waypoints(end,:)';
    goalRadius = 1;

    [~, pose] = ode45(@(t,y)derivative(diffDrive, y, ...
        myMobileRobotController(controller, y, goalPoint, goalRadius)), ...
        tVec, initPose);

    step_index = max(1, round(length(pose) * step_fraction));
    final_position = pose(step_index, 1:2);
    total_time = sampleTime * step_index;

    % === If playback (step_fraction ~ 1), plot entire trajectory ===
    if abs(step_fraction - 1.0) < 1e-5
        translations = [pose(:,1:2), zeros(length(pose),1)];
        rotations = axang2quat([repmat([0 0 1], length(pose), 1), pose(:,3)]);

        plot(waypoints(:,1), waypoints(:,2), "kx-", MarkerSize=10);
        hold on;

        plotTransforms(translations(1:10:end,:), rotations(1:10:end,:), ...
            MeshFilePath="groundvehicle.stl", MeshColor="g");

        axis equal;
        view(0, 90);
    end
end
