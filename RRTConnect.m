classdef RRTConnect < RRTBase
    methods(Access=public)
        % Constructor to initialize RRT with the environment
        function obj = RRTConnect(environment, varargin)
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
            differences = random_point - node_positions;  % Differences between random point and node positions
            distances = vecnorm(differences, 2, 2);  % Efficient vectorized norm calculation

            % 4. Find the nearest node
            [~, parent_index] = min(distances);
            nearest_node = obj.nodes(parent_index);

            % 5. Steer greedily towards the random point from the nearest node
            direction = differences(parent_index, :);  % Use precomputed difference for nearest node
            direction = direction / norm(direction);  % Normalize direction

            % Initialize the new position as the current nearest node position
            new_position = nearest_node.position;

            % Attempt to extend greedily towards the random point in multiple steps
            while true
                % Calculate the potential next position by moving in the direction
                next_position = new_position + direction * obj.step_size;

                % % Check if the next position would be outside the environment bounds or collide with an obstacle
                % if ~obj.environment.isValidPosition(next_position) || obj.environment.checkCollision(next_position)
                %     break;  % Stop if the next position is not feasible
                % end

                % Update the new position to the next position
                new_position = next_position;

                % Stop if the new position is close enough to the random point
                if norm(new_position - random_point) < obj.step_size
                    break;
                end
            end

            % Return the nearest node and the new node as output
            narargout{1} = nearest_node;
            narargout{2} = obj.createNode(new_position, parent_index);
        end

        function [start_nearest_node, goal_nearest_node, new_start_node, new_goal_node] = expandTrees(obj, start_tree, goal_tree)
            % 1. Select a random point in the environment
            random_point = obj.environment.randomPoint();

            % 2. Expand from the start tree
            start_node_positions = vertcat(start_tree.nodes(1:start_tree.number_nodes).position);
            start_differences = random_point - start_node_positions;
            start_distances = vecnorm(start_differences, 2, 2);
            [~, start_parent_index] = min(start_distances);
            start_nearest_node = start_tree.nodes(start_parent_index);

            start_direction = start_differences(start_parent_index, :);
            start_direction = start_direction / norm(start_direction);
            new_start_position = start_nearest_node.position;
            while true
                next_position = new_start_position + start_direction * obj.step_size;
                if ~obj.environment.isValidPosition(next_position) || obj.environment.checkCollision(next_position)
                    break;
                end
                new_start_position = next_position;
                if norm(new_start_position - random_point) < obj.step_size
                    break;
                end
            end
            new_start_node = obj.createNode(new_start_position, start_parent_index);

            % 3. Expand from the goal tree
            goal_node_positions = vertcat(goal_tree.nodes(1:goal_tree.number_nodes).position);
            goal_differences = random_point - goal_node_positions;
            goal_distances = vecnorm(goal_differences, 2, 2);
            [~, goal_parent_index] = min(goal_distances);
            goal_nearest_node = goal_tree.nodes(goal_parent_index);

            goal_direction = goal_differences(goal_parent_index, :);
            goal_direction = goal_direction / norm(goal_direction);
            new_goal_position = goal_nearest_node.position;
            while true
                next_position = new_goal_position + goal_direction * obj.step_size;
                if ~obj.environment.isValidPosition(next_position) || obj.environment.checkCollision(next_position)
                    break;
                end
                new_goal_position = next_position;
                if norm(new_goal_position - random_point) < obj.step_size
                    break;
                end
            end
            new_goal_node = obj.createNode(new_goal_position, goal_parent_index);

            % 4. Check if the two trees can be connected
            if norm(new_start_node.position - new_goal_node.position) < obj.step_size
                % Connect the two trees
                new_start_node.parent_index = new_goal_node.index;
            end
        end
    end
end