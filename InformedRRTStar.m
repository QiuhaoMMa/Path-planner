classdef InformedRRTStar < RRTS
    properties
        cMax = inf; % Initial c_max (used in informed sampling)
        Xsoln = []; % Set of solution nodes
    end

    methods (Access = public)
        % Constructor to initialize InformedRRTStar with the environment
        function obj = InformedRRTStar(environment, varargin)
            obj@RRTS(environment, varargin{:});
        end
        % Method to retrieve the final path from start to goal
        function final_path = getFinalPath(obj)
            % Initialize an empty array to store the path
            final_path = [];

            % Start from the goal node and trace back to the start node
            current_node = obj.nodes(end); % Assuming the goal node is the last one added
            while current_node.parent_index ~= 0
                final_path = [current_node.position; final_path]; % Prepend the current node position to the path
                current_node = obj.nodes(current_node.parent_index); % Move to the parent node
            end
            % Add the start node to the path
            final_path = [obj.nodes(1).position; final_path];
        end
    end

    methods (Access = protected)
        % Overriding the sample method to use informed sampling
        function xRand = sample(obj)
            if obj.cMax < inf
                xRand = obj.informedSample(obj.cMax);
            else
                % Regular random sampling if no solution exists
                xRand = obj.randomState();
            end
        end

        % Informed sampling method based on the algorithm provided
        function xRand = informedSample(obj, cMax)
            xStart = obj.environment.start;  % Get start position from environment
            xGoal = obj.environment.goal;    % Get goal position from environment
            cMin = norm(xGoal - xStart);
            xCenter = (xStart + xGoal) / 2;
            C = obj.rotationToWorldFrame(xStart, xGoal);
            r1 = cMax / 2;
            r = [r1; sqrt(r1^2 - cMin^2) / 2]; % Radius for ellipsoid

            % Generate a random point in a unit n-ball
            xBall = obj.sampleUnitNBall();
            
            % Map the sample to the ellipsoid in the environment
            xRand = C * diag(r) * xBall + xCenter;

            % Ensure the sample is within bounds
            if ~obj.isWithinBounds(xRand)
                xRand = obj.randomState(); % Fallback to random sampling if outside bounds
            end
        end

        % Rotational matrix to transform the unit ball to the ellipsoid frame
        function C = rotationToWorldFrame(~, xStart, xGoal)
            % Assuming 2D space for simplicity
            d = xGoal - xStart;
            theta = atan2(d(2), d(1)); % Angle of the line connecting start and goal
            C = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        end

        % Sample random point in a unit n-ball
        function xBall = sampleUnitNBall(obj)
            n = length(obj.environment.start); % Dimensionality based on environment's start
            xBall = randn(n, 1); % Random Gaussian vector
            xBall = xBall / norm(xBall) * rand^(1/n); % Normalize and scale to unit ball
        end

        % Add a new node and update the solution set Xsoln if in goal region
        function addNode(obj, node, draw)
            % Call the base class addNode method
            addNode@RRTS(obj, node, draw);

            % Update cMax if the node is in the goal region
            if obj.inGoalRegion(node.position)
                obj.Xsoln = [obj.Xsoln, node];
                obj.cMax = min([obj.cMax, node.distance2start]);
            end
        end

        % Method to check if a node is within the goal region
        function inGoal = inGoalRegion(obj, position)
            % Calculate the distance from the node's position to the goal
            xGoal = obj.environment.goal; % Get goal position from environment
            distanceToGoal = norm(position - xGoal);
            
            % Check if it's within the goal threshold (inherited from RRTBase)
            if obj.goal_threshold > 0
                inGoal = distanceToGoal <= obj.goal_threshold;
            else
                % Handle the case where goal_threshold is -1 or invalid
                inGoal = distanceToGoal == 0; % Direct hit at goal, or adjust as needed
            end
        end
    end
end
