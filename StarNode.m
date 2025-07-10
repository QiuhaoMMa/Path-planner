classdef StarNode < BasicNode
    properties
        index = 0;            % Node index in the tree, initialized to 0 by default
        distance2start = 0;    % Distance from the start node, initialized to 0 by default
    end

    methods
        % Constructor to initialize the node
        function obj = StarNode(position, parent_index)
            % Call the parent constructor (BasicNode) to initialize common properties
            obj@BasicNode(position, parent_index);
        end
    end
end
