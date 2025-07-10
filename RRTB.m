classdef RRTB<RRTBase&handle
    properties
        RRTType = @RRTS       % Function handle to specify the RRT algorithm type
        RRTs                  % Store RRT instances (one for start, one for goal)
        meetNodes = []        % Store the nodes where the two trees meet
    end
    methods(Access=public)
        % Constructor to initialize RRT with the environment
        function obj = RRTB(environment, varargin)
            obj@RRTBase(environment, varargin{:});

            % Parse optional key-value arguments
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                if isprop(obj, key)
                    obj.(key) = value;
                end
            end

            % Initialize two RRT trees, one from the start and one from the goal
            environment0=environment.copy();
            environment1=environment.copy();
            environment1.goal=environment.start;
            environment1.start=environment.goal;

            obj.RRTs{1} = obj.RRTType(environment0, varargin{:});  % Use RRTType handle
            obj.RRTs{2} = obj.RRTType(environment1, varargin{:});
        end


        % Reconstruct the path once the trees meet
        function path_nodes = reconstructPath(obj)
            if numel(obj.meetNodes) == 1
                path_nodes = obj.RRTs{obj.meetNodes}.reconstructPath();
            else
                path_nodes = cell(1, 2);  % One for each tree
                for i = 1:size(obj.meetNodes, 1)
                    treeIdx = obj.meetNodes(i, 1);
                    nodeIdx = obj.meetNodes(i, 2);
                    path_nodes{treeIdx} = obj.RRTs{treeIdx}.reconstructPath(nodeIdx);
                end
            end
        end


        % Get the total path length from start to goal
        function L = getPathLength(obj)
            L = 0;
            obj.reconstructPath();  % Ensure the path is reconstructed

            if numel(obj.meetNodes) == 1
                L = obj.RRTs{obj.meetNodes}.getPathLength();
            else
                for i = 1:size(obj.meetNodes, 1)
                    treeIdx = obj.meetNodes(i, 1);
                    nodeIdx = obj.meetNodes(i, 2);
                    L = L + obj.RRTs{treeIdx}.getPathLength(nodeIdx);
                end
            end
        end

        % Plot the reconstructed path
        function plotPath(obj)
            path_nodes = obj.reconstructPath();
            COLORS = ['g', 'r', 'c', 'y'];  % Colors for the different trees
            if numel(obj.meetNodes)==1
                obj.RRTs{obj.meetNodes}.plotPath(COLORS(obj.meetNodes));
                return;
            end
            for i = 1:size(obj.meetNodes, 1)
                treeIdx = obj.meetNodes(i, 1);
                nodeIdx = obj.meetNodes(i, 2);
                obj.RRTs{treeIdx}.plotPath(nodeIdx, COLORS(treeIdx));
            end
        end
    end

    methods (Access=protected)
        % Override the selectCandidateNode method
        function selectCandidateNode(obj)
            % Custom behavior for selecting nodes in bidirectional RRTs can be added here
        end


        % Method to extend the trees and check if they meet
        function isMeet = extendTree(obj, draw)
            isMeet = false;
            obj.meetNodes = [];  % Reset meetNodes

            % Step 1: Try extending both trees individually
            for i = 1:numel(obj.RRTs)
                if obj.RRTs{i}.extendTree(draw)
                    obj.meetNodes = i;
                    isMeet = true;
                    obj.computation_time=toc(obj.start_time);
                    return;
                end
            end

            % Step 2: Check if the two trees meet
            for i=1:numel(obj.RRTs)
                RRT1=obj.RRTs{i};
                for j=1:numel(obj.RRTs)
                    if i~=j
                        RRT2=obj.RRTs{j};

                        % Vectorized distance calculation
                        positions = vertcat(RRT1.nodes(1:RRT1.number_nodes).position);
                        distances = vecnorm(positions - RRT2.nodes(RRT2.number_nodes).position, 2, 2);

                        % Find the first node in RRT1 close enough to the last node in RRT2
                        meetIdx = find(distances < obj.environment.resolution, 1);
                        if ~isempty(meetIdx)
                            if ~obj.environment.checkCollision(positions(meetIdx,:), RRT2.nodes(RRT2.number_nodes).position)
                                obj.meetNodes = [i meetIdx; j RRT2.number_nodes];
                                isMeet = true;
                                obj.path_length=obj.getPathLength();
                                obj.computation_time=toc(obj.start_time);
                                return;
                            end
                        end
                    end
                end
            end
        end

    end
end
