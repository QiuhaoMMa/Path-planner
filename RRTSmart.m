classdef RRTSmart < RRTS
    properties
        Zbeacons = []       % Beacons for smart sampling
        beaconInterval = 10 % Interval for smart sampling using beacons
        initialPathFound = false % Flag to indicate if an initial path is found
    end

    methods(Access=public)
        % Constructor to initialize RRT*Smart with the environment
        function obj = RRTSmart(environment, varargin)
            obj@RRTS(environment, varargin{:});

            % Parse optional key-value arguments for beacons and intervals
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                if isprop(obj, key)
                    obj.(key) = value;
                end
            end
        end

        % Override the main planning function to include smart sampling and path optimization
        function plan(obj, max_iterations)
            obj.initializeTree();

            % Insert the initial node into the tree
            obj.insertNode([], obj.environment.start);

            for i = 1:max_iterations
                if obj.useSmartSampling(i)
                    Z_rand = obj.smartSample();
                else
                    Z_rand = obj.randomSample();
                end

                Z_nearest = obj.getNearestNode(Z_rand);
                [Z_new, u_new, T_new] = obj.steer(Z_nearest, Z_rand);

                if obj.environment.isObstacleFree(Z_new)
                    Z_near = obj.getNearbyNodes(Z_new);
                    Z_min = obj.chooseParent(Z_near, Z_nearest, Z_new);

                    obj.insertNode(Z_min, Z_new);
                    obj.rewireTree(Z_near, Z_min, Z_new);

                    if ~obj.initialPathFound && obj.environment.isGoalReached(Z_new)
                        obj.initialPathFound = true;
                        [~, directCost] = obj.pathOptimization();
                        obj.Zbeacons = obj.pathOptimization();
                    end

                    % If initial path was found, keep optimizing the path
                    if obj.initialPathFound
                        [~, directCostNew] = obj.pathOptimization();
                        if directCostNew < directCost
                            obj.Zbeacons = obj.pathOptimization(); % Update beacons
                        end
                    end
                end
            end
        end
    end

    methods(Access=private)
        % Check whether smart sampling should be used
        function flag = useSmartSampling(obj, iteration)
            flag = mod(iteration, obj.beaconInterval) == 0 && ~isempty(obj.Zbeacons);
        end

        % Smart sampling function that samples from beacons
        function Z_rand = smartSample(obj)
            % Choose a random beacon from the list of Zbeacons
            beaconIdx = randi(length(obj.Zbeacons));
            Z_rand = obj.Zbeacons(beaconIdx);
        end

        % Path optimization method
        function [T_new, directCost] = pathOptimization(obj)
            % Path optimization logic can be an external optimization algorithm
            T_new = obj.findPath(); % Find the current best path
            directCost = obj.computePathCost(T_new);
        end
    end
end
