classdef RRTS < RRT
    properties
        search_radius  = 20
    end

    methods(Access=public)
        % Constructor to initialize RRTS with the environment
        function obj = RRTS(environment, varargin)
            obj@RRT(environment, varargin{:});

            % Parse optional key-value arguments
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                if isprop(obj, key)
                    obj.(key) = value;
                end
            end
        end
    end

    methods (Access=protected)
        % Factory method to create a node (can be overridden by subclasses)
        function node = createNode(obj, position, parent_index)
            node = StarNode(position, parent_index);
            node.index=obj.number_nodes+1;
        end


        % Add a new node to the RRTS tree
        function addNode(obj, node, draw)
            parent_node = obj.nodes(node.parent_index);

            % Update distance to start
            node.distance2start = parent_node.distance2start + norm(node.position - parent_node.position);

            % Add the node using the base method
            addNode@RRTBase(obj, node, draw);

            % Rewire nearby nodes for optimization
            obj.rewire(node);
        end

        function [potential_parents_idx] = rewire(obj, node)
            % Extract all positions and distances for the current nodes
            parent_positions = vertcat(obj.nodes(1:obj.number_nodes).position);  % Extract all positions
            differences = parent_positions - node.position;
            distances = vecnorm(differences, 2, 2)';  % Compute distances

            % Find nearby nodes within the search radius (filter nodes before collision checks)
            search_radius_limit = obj.search_radius * obj.environment.resolution;
            potential_parents_idx = find(distances > 0 & distances < search_radius_limit);

            if isempty(potential_parents_idx)
                return;
            end

            % Extract distances to start and positions of potential parents
            parent_distances2start = [obj.nodes(potential_parents_idx).distance2start];
            parent_positions = parent_positions(potential_parents_idx, :);  % Filter for potential parents' positions

            % Compute new distances to start if rewired through the new node
            distances2startNew = parent_distances2start + distances(potential_parents_idx);

            % Check rewiring conditions in a vectorized manner (filter first, collision check later)
            rewire_candidates = (distances2startNew < node.distance2start);

            if any(rewire_candidates)
                % Filter potential parents to those that satisfy the rewiring condition
                candidates_idx = potential_parents_idx(rewire_candidates);
                candidates_positions = parent_positions(rewire_candidates, :);
                candidates_distances2startNew = distances2startNew(rewire_candidates);

                % Perform collision checks only on valid candidates
                collision_checks = arrayfun(@(i) obj.environment.checkCollision(candidates_positions(i, :), node.position), 1:length(candidates_idx));

                % Keep only the candidates with no collisions
                valid_candidates = candidates_idx(~collision_checks);
                valid_distances2startNew = candidates_distances2startNew(~collision_checks);

                if ~isempty(valid_candidates)
                    % Select the best candidate that minimizes the distance to start
                    [~, best_idx] = min(valid_distances2startNew);
                    best_parent_idx = valid_candidates(best_idx);

                    % Rewire the new node to the best parent
                    node.parent_index = obj.nodes(best_parent_idx).index;
                    node.distance2start = valid_distances2startNew(best_idx);
                end
            end

            % --- Phase 2: Rewire existing nodes to the new node if beneficial ---

            % Compute new distances to start if children are rewired through the new node
            child_positions = parent_positions;
            child_distances2start_new = distances(potential_parents_idx) + node.distance2start;

            % Extract the current distance to start for all potential children
            child_distances2start_old = [obj.nodes(potential_parents_idx).distance2start];

            % Only consider candidates where rewiring would reduce the distance to start
            rewire_candidates = (child_distances2start_new < child_distances2start_old);

            if any(rewire_candidates)
                % Perform collision checks for valid children only
                valid_candidates = find(rewire_candidates);
                collision_checks = arrayfun(@(i) obj.environment.checkCollision(node.position, child_positions(valid_candidates(i), :)), 1:length(valid_candidates));

                % Rewire the valid children
                for i = valid_candidates(~collision_checks)
                    child_idx = potential_parents_idx(i);
                    obj.nodes(child_idx).parent_index = node.index;
                    obj.nodes(child_idx).distance2start = child_distances2start_new(i);
                end
            end
        end

    end
end
