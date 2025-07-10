classdef LazyRRT < RRT
    methods(Access=public)
        function obj = LazyRRT(environment, varargin)

            obj@RRT(environment, varargin{:});
        end
    end

    methods(Access=protected)
        function narargout = selectCandidateNode(obj, varargin)

            random_point = obj.environment.randomPoint();
            node_positions = vertcat(obj.nodes(1:obj.number_nodes).position);  
            differences = random_point - node_positions;
            distances = vecnorm(differences, 2, 2);  
            [~, parent_index] = min(distances);
            nearest_node = obj.nodes(parent_index);


            direction = differences(parent_index, :);
            direction = direction / norm(direction);
            new_position = nearest_node.position + direction * obj.step_size;
            narargout{1} = nearest_node;
            narargout{2} = obj.createNode(new_position, parent_index);
        end

        function finalPath = generatePath(obj)

            tree = obj.initializeTree();
            for i = 1:obj.maxIterations
                [~, newNode] = obj.selectCandidateNode();
                tree = [tree, newNode];  % 添加节点到树中
                if obj.isGoalReached(newNode)
                    finalPath = obj.traceBackPath(newNode);
                    return;
                end
            end
            finalPath = []; 
        end
    end
end