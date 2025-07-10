classdef RRTBase < handle & matlab.mixin.Copyable
    properties (Access=public)
        id
        environment     % Environment object (goal, start, boundary, obstacles)
        nodes = []      % Store the nodes (tree structure)
        step_size = -1 % Step size for node expansion
        max_iterations = 1e6  % Maximum number of iterations
        memory_allocation=5e2
        number_nodes = 1       % Total number of nodes
        num_iter=0
        goal_threshold=-1;


        h_old=[];
        drawing=0;
        axis_handle=[]
    end

    properties (Access=protected)
        start_time=0;
    end

    properties (Access=public)
        computation_time=0;
        path_length=0;
    end

    methods (Abstract,Access=protected)
        selectCandidateNode(obj,varargin)
    end


    methods(Access=public)
        % Constructor to initialize RRT with the environment
        function obj = RRTBase(environment, varargin)
            obj.start_time=tic;
            obj.environment=environment;
            obj.axis_handle=obj.environment.axis_handle;
            % Parse optional key-value arguments
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                if isprop(obj, key)
                    obj.(key) = value;
                end
            end
            obj.id=rand();

            if isempty(obj.axis_handle)
                obj.environment.plot;
                obj.axis_handle=obj.environment.axis_handle;
            end

            % Initialize the first node at the start position
            startNode = obj.createNode(environment.start, 0);
            obj.nodes = startNode;

            if (obj.step_size<0)
                obj.step_size=min(obj.environment.resolution);
            end
            if (obj.goal_threshold<0)
                obj.goal_threshold=obj.step_size;
            end
        end



        % Solve the RRT problem by expanding the tree
        function goal_reached = solve(obj, drawing)
            for iter = 1:obj.max_iterations
                goal_reached = obj.extendTree(drawing);
                if goal_reached
                    obj.num_iter = iter;
                    if (drawing>0)
                        obj.plotPath();
                    end
                    break;  % Stop further iterations once the goal is reached
                end
            end
        end




        function children_indices = getChildren(obj, node)
            % Instead of iterating, use logical indexing to find all children at once
            children_indices = find([obj.nodes.parent_index] == node.index);
        end


        function path_nodes = reconstructPath(obj, varargin)
            if nargin == 1
                nodeIdx = obj.number_nodes;  % Start from the last node
            else
                nodeIdx = varargin{1};  % Start from a specific node
            end

            % Estimate the maximum possible path length as the number of nodes
            path_nodes = zeros(obj.number_nodes+1, 1);  % Preallocate memory
            count = 0;  % Counter to track the number of nodes in the path

            % Trace back through the tree until we reach the root
            while nodeIdx ~= 0
                count = count + 1;  % Increment path length counter
                path_nodes(count) = nodeIdx;  % Store the current node index
                nodeIdx = obj.nodes(nodeIdx).parent_index;  % Move to the parent node
            end

            % Trim unused space and reverse the path to get the correct order
            path_nodes = flipud(path_nodes(1:count));  % Only keep valid nodes
        end



        function L = getPathLength(obj, varargin)
            path_nodes = obj.reconstructPath(varargin{:});
            positions = vertcat(obj.nodes(path_nodes).position);  % Vectorized position extraction
            L = sum(vecnorm(diff(positions), 2, 2));  % Efficient vectorized distance computation
        end


        function plotPath(obj, varargin)
            color = 'r';
            path_nodes = obj.reconstructPath();
            if nargin > 1
                path_nodes = obj.reconstructPath(varargin{1});
            end
            if nargin > 2
                color = varargin{2};
            end

            if ~ishandle(obj.axis_handle)
                obj.environment.plot;
                obj.axis_handle=obj.environment.axis_handle;
            end

            % Plot the path without repeatedly calling axis
            for i = 1:length(path_nodes) - 1
                plot(obj.axis_handle,[obj.nodes(path_nodes(i)).position(1), obj.nodes(path_nodes(i + 1)).position(1)], ...
                    [obj.nodes(path_nodes(i)).position(2), obj.nodes(path_nodes(i + 1)).position(2)], color, 'LineWidth', 2);
            end
        end

    end

    methods (Access=protected)
        % Factory method to create a node (can be overridden by subclasses)
        function node = createNode(~, position, parent_index)
            node = BasicNode(position, parent_index);
        end

        function collisionCallBack(obj)
            return;
        end

        function goal_reached = extendTree(obj, drawing)
            goal_reached = false;
            PCnodes = obj.selectCandidateNode();


            circle_radius = 1;  % 根据 AGV 尺寸调整这个值

            % Check for collision only if the new position is inside the boundary
            if obj.environment.isInSpaceBoundary(PCnodes{2}.position)
                if ~obj.environment.checkCollision(PCnodes{1}.position, PCnodes{2}.position)


                    % 新增：节点作为一个圆形进行碰撞检测
                    circle.center = PCnodes{2}.position;  % 圆心是节点的位置
                    circle.radius = circle_radius;        % 定义的半径
                    if ~obj.environment.checkCollision([], circle)

                        obj.addNode(PCnodes{2}, drawing);

                        if obj.environment.reachedGoal(PCnodes{2}.position,obj.goal_threshold)
                            obj.computation_time=toc(obj.start_time);
                            obj.path_length=obj.getPathLength();
                            goal_reached = true;
                        end
                    else
                        % 如果圆形与障碍物碰撞，触发碰撞回调
                        obj.collisionCallBack();
                    end
                else
                    obj.collisionCallBack();
                end
            end
        end

        % Add a new node to the RRT tree
        function addNode(obj, new_node, drawing)
            obj.number_nodes = obj.number_nodes + 1;
            if length(obj.nodes) < obj.number_nodes
                obj.nodes = [obj.nodes; repmat(new_node, obj.memory_allocation, 1)];  % Preallocate in chunks
            end
            obj.nodes(obj.number_nodes) = new_node;  % Assign new node

            % Optionally draw the path from the parent to the new node
            if drawing>0
                if ~ishandle(obj.axis_handle)
                    obj.environment.plot;
                    obj.axis_handle=obj.environment.axis_handle;
                end

                if obj.number_nodes==2
                    obj.drawing=drawing;
                end

                if ishandle(obj.h_old)
                    delete(obj.h_old);  % Avoid redrawing over existing points
                end

                parent_node = obj.nodes(new_node.parent_index);
                % Draw the line and new point in a single call to reduce overhead
                plot(obj.axis_handle,[parent_node.position(1), new_node.position(1)], ...
                    [parent_node.position(2), new_node.position(2)], 'b-');
                obj.h_old = plot(obj.axis_handle,new_node.position(1), new_node.position(2), 'r.', 'MarkerSize', 40);

                if mod(obj.number_nodes-2,drawing)==0
                    drawnow;
                    % drawnow limitrate;  % Throttle drawing rate for better performance
                end
            end
        end





        function trimBranch(obj, node)
            % Use a stack for iterative trimming instead of recursion
            stack = node;
            while ~isempty(stack)
                current_node = stack(end);
                stack(end) = [];  % Pop the current node

                % Get all children of the current node and add to the stack
                children_indices = obj.getChildren(current_node);
                stack = [stack; obj.nodes(children_indices)];  % Add children to the stack

                % Remove the current node
                obj.nodes(current_node.index) = [];
            end
        end

    end

    methods (Static)
        function RRT=connectTrees(RRT1, node_index, new_index)
            N=ceil(log10(RRT1.number_nodes))+1;
            RRT=RRT1.copy();
            % Step 1: Get the node from RRT1 that needs to be connected
            parent_index = RRT.nodes(node_index).parent_index;
            pparent_index = RRT.nodes(parent_index).parent_index;
            RRT.nodes(parent_index).parent_index=new_index;


            % Set the parent index of nodeA to nodeB's index in RRT2
            node.parent_index = N+new_index;  % Or use a different indexing system if needed

            % For example, you may want to propagate changes to other nodes connected to nodeA
            RRTBase.updateChildrenNodes(RRT,  N,N+new_index);
        end

        % Function to update child nodes based on the new parent connection
        function updateChildrenNodes(RRT, N,parent_index)
            % Loop through all nodes to find children of the updated parent node

            S=vertcat(RRT.nodes(:).parent_index==parent_index);
            RRT.nodes(S).parent_index
            RRT.nodes.parent_index
            for i = 1:numel(RRT.nodes)
                if RRT.nodes(i).parent_index == parent_index
                    % Update the distance2start for the child node
                    % Recursively update child nodes of this node
                    RRTBase.updateChildrenNodes(RRT, i);
                end
            end
        end
    end
end
