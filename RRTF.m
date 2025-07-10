classdef RRTF
    properties
        nodes  % Stores nodes (positions)
        parents  % Stores parent relationships
        stepSize  % Step size for expansion
    end
    
    methods
        function obj = RRTF(start, stepSize)
            obj.nodes = start;  % Start point as the root node
            obj.parents = 0;  % Root node has no parent
            obj.stepSize = stepSize;  % Step size for expanding the tree
        end
        
        function nearestIdx = findNearestNode(obj, randomPoint)
            distances = sqrt(sum((obj.nodes - randomPoint).^2, 2));
            [~, nearestIdx] = min(distances);
        end
        
        function newNode = steer(obj, fromNode, toNode)
            direction = toNode - fromNode;
            distance = norm(direction);
            newNode = fromNode + (direction / distance) * min(obj.stepSize, distance);
        end
        
        function obj = addNode(obj, newNode, parentIdx)
            obj.nodes = [obj.nodes; newNode];
            obj.parents = [obj.parents; parentIdx];
        end
        
        % New method: Update RRT based on PSO/PF particle paths
        function obj = updateFromParticles(obj, particlePaths, mergeThreshold)
            for i = 1:numel(particlePaths)
                particlePath = particlePaths{i};
                for j = 1:size(particlePath, 1)
                    % Check if the particle's path node is close to any existing RRT node
                    distances = sqrt(sum((obj.nodes - particlePath(j, :)).^2, 2));
                    if all(distances > mergeThreshold)
                        % If not close to any existing RRT node, add it to the RRT
                        nearestIdx = obj.findNearestNode(particlePath(j, :));
                        obj = obj.addNode(particlePath(j, :), nearestIdx);
                    end
                end
            end
        end
        
        % New method: Reconstruct path from a node index to the root
        function path = reconstructRRTPath(obj, nodeIdx)
            path = [];
            while nodeIdx ~= 0
                path = [obj.nodes(nodeIdx, :); path];  % Prepend the node to the path
                nodeIdx = obj.parents(nodeIdx);  % Move to the parent node
            end
        end
    end
end
