classdef SmartNode < StarNode
    properties
        pdf_direction = []      % Matrix to store the PDF for direction
        branch = 0;            % Tracks the branch of the node
        occupancy = 0;         % Occupancy of the node
        distance2goal = inf;   % Distance to the goal node
    end
    
    methods
        % Constructor to initialize the node
        function obj = SmartNode(position, parent_index, distance2goal,resolution)
            % Call the parent constructor
            obj@StarNode(position, parent_index);
            
            % Initialize the pdf_direction only if it's not already created
            if isempty(obj.pdf_direction)
                obj.pdf_direction = PDFMatrix('boundary', [-1 1; -1 1], 'resolution', resolution);
            end
            
            % Set distance to goal
            obj.distance2goal = distance2goal;
        end
        
        % Add potential direction with reward
        function obj = potentialDirection(obj, newDirection, reward)
            obj.pdf_direction.addDirectionSample(newDirection, 2 * reward, 1e1);  % Simplified function call
        end
        
        % Record visited direction with penalty
        function obj = visitedDirection(obj, newDirection, penalty)
            obj.pdf_direction.addDirectionSample(newDirection, -10 * penalty, 1e4);  % Simplified function call
        end
        
        % Record collision direction with penalty
        function obj = collisionDirection(obj, newDirection, penalty)
            obj.pdf_direction.addDirectionSample(newDirection, -10 * penalty, 1e3);  % Simplified function call
        end
        
        % Get direction, with optional cutoff argument
        function value = getDirection(obj, varargin)
            if nargin > 1
                value = obj.pdf_direction.getSample(varargin{1} / 100);  % Use custom cutoff if provided
            else
                value = obj.pdf_direction.getSample(95 / 100);  % Default to 95%
            end
        end
    end
end
