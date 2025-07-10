classdef RRT < RRTBase
    methods(Access=public)
        % Constructor to initialize RRT with the environment
        function obj = RRT(environment, varargin)
            obj@RRTBase(environment, varargin{:});
        end
    end

    methods (Access=protected)
        function narargout = selectCandidateNode(obj, varargin)
            % 1. Select a random point in the environment
            random_point = obj.environment.randomPoint();

            % 2. Vectorize the position extraction for all nodes
            node_positions = vertcat(obj.nodes(1:obj.number_nodes).position);  % Extract all node positions in one go

            % 3. Calculate the distance between all nodes and the random point (vectorized)
            differences = random_point-node_positions ;  % Differences between random point and node positions
            distances = vecnorm(differences, 2, 2);  % Efficient vectorized norm calculation

            % 4. Find the nearest node
            [~, parent_index] = min(distances);
            nearest_node = obj.nodes(parent_index);

            % 5. Steer towards the random point from the nearest node
            direction = differences(parent_index, :);  % Use precomputed difference for nearest node
            direction = direction / norm(direction);  % Normalize direction
            new_position = nearest_node.position + direction * obj.step_size;  % Move towards the point

            % Return the nearest node and the new node as output
            narargout{1} = nearest_node;
            narargout{2} = obj.createNode(new_position, parent_index);
        end
    end
end
