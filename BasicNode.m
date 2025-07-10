classdef BasicNode < matlab.mixin.Copyable
    properties
        parent_index = 0                    % Track the parent index of each node
        position =zeros(1, 2);              % Node position (preallocate as row vector)
    end
    
    methods
        % Constructor to initialize the node
        function obj = BasicNode(position, parent_index)
            obj.position = position;  % Initialize the position
            obj.parent_index = parent_index;  % Store the parent_index node
        end
        
    end
end
