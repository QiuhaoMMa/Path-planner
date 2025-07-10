classdef RRTP < RRTS
    properties
        distance2goal = 0;  % Shortest distance from the start node to the goal
        current2goal = 0;   % Distance of the current node to the goal
        
        occupancy_pdf_resolution=10;
        steering_resolution=0.2;
        
        % Structure to store the reward weights for different factors
        reward = struct('direction', 1, ...  % Reward weight for direction
            'position', 1, ...   % Reward weight for position
            'velocity', 1, ...   % Reward weight for velocity
            'distance2goal', 1, ...  % Reward weight for distance to the goal
            'occupied', 1);  % Reward weight for occupancy of nodes
        
        candidate_node;    % Node that is currently being evaluated or expanded
        occupancy_pdf;      % Probability Density Function (PDF) matrix for node occupancy
        trends = [];       % Buffer to track trends and changes in the tree growth
        buffer = [];       % Buffer for storing occupancy-related data for trend analysis
        successfulRate = 1; % Success rate for expanding nodes, influences tree growth
        
        best_occupied_node = eps;  % Stores the highest occupancy value in the tree
        worst_occupied_node = 1;   % Stores the lowest occupancy value in the tree
    end
    
    methods(Access=public)
        % Constructor to initialize RRT with the environment
        function obj = RRTP(environment, varargin)
            % Call the parent constructor to set up the RRTS environment
            obj@RRTS(environment, varargin{:});
            
            % Parse optional key-value arguments for initialization
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                if isprop(obj, key)
                    obj.(key) = value;  % Set the property values dynamically
                end
            end
            
            obj.current2goal = 0;  % Initialize the current node's distance to goal
            
            % Initialize buffer and trend tracker with a default buffer
            obj.buffer = [0;0;0];
            obj.trends = Buffer(100, 1, obj.buffer);  % Track trends over a sliding window
            
            % Initialize occupancy PDF for tracking how nodes occupy space in the environment
            obj.occupancy_pdf = PDFMatrix('boundary', obj.environment.boundary, ...
                'resolution', obj.occupancy_pdf_resolution * ones(1, obj.environment.dimension));
        end
    end
    
    methods (Access=protected)
        % Factory method to create a new node
        function node = createNode(obj, position, parent_index)
            % Create a SmartNode, initializing it with position, parent index, and normalized distance to the goal
            node = SmartNode(position, parent_index, ...
                obj.environment.distanceToGoal(position) / obj.environment.max_distance2goal,obj.steering_resolution);
            
            % Calculate the direction from the start to the goal and add direction potential to the node
            direction_start2goal = obj.environment.directionToGoal(position);
            node.potentialDirection(direction_start2goal, obj.reward.direction);
            
            node.index=obj.number_nodes+1;  % Set node index to the total number of nodes + 1
        end
        
        % Method to select a candidate node for tree expansion
        function narargout = selectCandidateNode(obj, varargin)
            % Weighting factor to combine occupancy and distance-based rewards
            weight_ocp1dis = 0.9;
            
            % Calculate the reward for each node based on occupancy and distance to goal
            occupancies = vertcat(obj.nodes(1:obj.number_nodes).occupancy);
            distances2goal = vertcat(obj.nodes(1:obj.number_nodes).distance2goal);
            distances2start = vertcat(obj.nodes(1:obj.number_nodes).distance2start);
            
            obj.trends.insertData([mean(occupancies);max(distances2goal);max(distances2start)]);
            W=[1 0.5 0.5];
            W=W/norm(W);
            P=obj.trends.growingRate();
            Pn=mean(sfunc(W.*2.^P(1,:), 'sigma', [10 0]),2);
            
            rewards = (sfunc(occupancies, 'sigman', [0.1, (obj.best_occupied_node + obj.worst_occupied_node) / 2])).^(weight_ocp1dis) .* ...
                (1 - sfunc(0.99 * distances2goal + 1e-6 * distances2start, 'sigman', [1, 1])).^(1 - weight_ocp1dis);
            
            % Select a parent node based on a biased random selection using the rewards
            [parent_index, ~] = biasedRandomSelection(rewards, 1, 10, 0.95);
            
            parent_node = obj.nodes(parent_index);
            
            % Calculate the candidate direction by interpolating between a random direction and the parent node's direction
            direction = parent_node.getDirection(100);
            randomDir = randn(1, obj.environment.dimension);
            randomDir = randomDir / norm(randomDir);
            direction = RRTP.Slerp(randomDir, direction, 0.7);  % Slerp for smooth interpolation
            
            % Compute the candidate position based on the direction and step size
            candidate_position = parent_node.position + direction * obj.step_size * obj.successfulRate;
            
            % Create the candidate node and set up direction information
            obj.candidate_node = obj.createNode(candidate_position, parent_index);
            parent_node.visitedDirection(direction, obj.reward.direction);
            obj.candidate_node.visitedDirection(-direction, obj.reward.direction);
            
            % Mix the direction PDF of the new node with the parent node's direction PDF
            weight_newVsParent =0.9999;
            obj.candidate_node.pdf_direction.matrix = ...
                weight_newVsParent * obj.candidate_node.pdf_direction.matrix + ...
                (1 - weight_newVsParent) * parent_node.pdf_direction.matrix;
            
            % Update the occupancy PDF based on the new candidate node
            obj.occupancy_pdf.addSample(parent_node.position, -obj.reward.position, ...
                direction, -obj.reward.velocity,....
                10 * max(obj.occupancy_pdf.resolution)^2);
            obj.occupancy_pdf.addSample(obj.candidate_node.position, -obj.reward.position, ...
                -direction, -obj.reward.velocity,...
                10 * max(obj.occupancy_pdf.resolution)^2);
            
            % Update occupancy values for nodes
            obj.updateOccupancy();
            
            % Return the parent node and candidate node
            narargout{1} = parent_node;
            narargout{2} = obj.candidate_node;
        end
        
        
        % Method to extend the RRT by expanding the tree
        function goal_reached = extendTree(obj, draw)
            goal_reached = extendTree@RRTS(obj, draw);  % Call parent method to extend the tree
            
            % Plot the occupancy PDF if drawing is enabled and it's time to plot
            if obj.drawing > 0 && (mod(obj.number_nodes, obj.drawing) == 0)
                obj.occupancy_pdf.plot(0.15);  % Plot updated occupancy PDF
            end
        end
        
        % Callback method to handle collisions during tree expansion
        function collisionCallBack(obj)
            parent_node = obj.nodes(obj.candidate_node.parent_index);  % Get the parent node of the candidate node
            
            % Update occupancy PDF based on the candidate node's position and direction
            obj.occupancy_pdf.addSample( ...
                obj.candidate_node.position, -10 * obj.reward.position, ...
                -obj.candidate_node.getDirection(95), -10 * obj.reward.velocity, 100 * max(obj.occupancy_pdf.resolution)^2);
            
            % Mark the parent node's direction as a collision
            parent_node.collisionDirection(obj.candidate_node.getDirection(95), obj.reward.direction);
        end
        
        % Update the occupancy values for all nodes based on the occupancy PDF
        function updateOccupancy(obj)
            % Loop through all nodes and update their occupancy values
            for i = 1:obj.number_nodes
                obj.nodes(i).occupancy = obj.occupancy_pdf.getNormalizedValueByPosition(obj.nodes(i).position);
                
                % Track the best and worst occupancy values encountered
                obj.best_occupied_node = max(obj.nodes(i).occupancy, obj.best_occupied_node);
                obj.worst_occupied_node = min(obj.nodes(i).occupancy, obj.worst_occupied_node);
            end
        end
        
        
        % Add a new node to the RRT tree
        function addNode(obj, candidate_node, draw)
            % Get the parent node of the candidate node
            parent_node = obj.nodes(candidate_node.parent_index);
            
            % Plot occupancy if drawing is enabled
            if obj.drawing > 0 && (mod(obj.number_nodes, obj.drawing) == 0)
                obj.occupancy_pdf.plot(0.15);
            end
            
            % Call the parent method to actually add the node to the tree
            addNode@RRTS(obj, candidate_node, draw);
            
            % Increment the branch count of the parent node
            parent_node.branch = parent_node.branch + 1;
        end
        
        % Method to directly move a candidate node toward the goal
        function directGo(obj, candidate_node)
            direction = Environment.Direction(candidate_node.position, obj.environment.goal);  % Calculate direction
            
            % Try to move the candidate node towards the goal, checking for collisions
            for n = 0:3
                new_position = candidate_node.position + direction * obj.distance2goal / 2^n;
                if ~obj.environment.checkCollision(obj.nodes(candidate_node.parent_index).position, new_position)
                    candidate_node.position = new_position;
                    return;
                end
            end
        end
        
        % Helper function to retrieve values from a field in the node array
        function y = getInMatrix(obj, field)
            y = cell2mat(arrayfun(@(node) node.(field), obj.nodes, 'UniformOutput', false));
        end
        
    end
    
    methods (Static)
        % Spherical Linear Interpolation (Slerp) for direction vectors
        function v_interp = Slerp(v1, v2, t)
            theta = acos(dot(v1, v2));  % Angle between the two vectors
            v_interp = (sin((1 - t) * theta) / sin(theta)) * v1 + (sin(t * theta) / sin(theta)) * v2;
            v_interp = v_interp / norm(v_interp);  % Normalize the result
        end
    end
end
