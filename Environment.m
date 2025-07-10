classdef Environment < matlab.mixin.Copyable
    properties
        goal           % Goal position
        start          % Start position
        resolution = 5 % Threshold for reaching the goal
        dimension      % Dimension of the environment (e.g., 2 for 2D)
        boundary       % Boundary of the environment
        obstacles = [] % Array of Obstacle objects in the environment
        max_distance2goal = inf
        boundary_corners % Precomputed corners of the boundary
        axis_handle
        add_image = false % Option to add image
    end
    
    methods
        % Constructor to initialize the environment
        function obj = Environment(start, goal, boundary, varargin)
            obj.goal = goal;
            obj.start = start; % Start point as the root node
            obj.boundary = boundary;  % Initialize the boundary
            obj.dimension = size(boundary, 1);  % Set the dimension
            obj.boundary_corners = obj.computeBoundaryCorners();  % Precompute boundary corners
            obj.max_distance2goal = obj.getLongestDistance(obj.start);
            
            % Handle optional arguments (e.g., resolution, obstacles, add_image)
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                obj.(key) = value;
            end
            
            % Check if start and goal points are within boundaries
            if (~obj.isInSpaceBoundary(start))
                error("The start point is out of the config space.");
            end
            
            if (~obj.isInSpaceBoundary(goal))
                error("The goal point is out of the config space.");
            end
        end

        function corners = computeBoundaryCorners(obj)
            % Precompute all corners of the environment boundary
            num_corners = 2^obj.dimension;  % 2^dimension corners in an n-dimensional space
            corners = zeros(num_corners, obj.dimension);

            % Compute the corners of the boundary by adjusting the min and max of each dimension
            for i = 1:num_corners
                bin_str = dec2bin(i-1, obj.dimension);  % Binary representation
                for j = 1:obj.dimension
                    if bin_str(j) == '0'
                        corners(i, j) = obj.boundary(j, 1);  % Min boundary
                    else
                        corners(i, j) = obj.boundary(j, 2);  % Max boundary
                    end
                end
            end
        end
        
        function max_distance = getLongestDistance(obj, point)
            % Use precomputed corners of the boundary to calculate the longest distance
            distances = sqrt(sum((obj.boundary_corners - point).^2, 2));
            max_distance = max(distances);
        end
        
        % Check if a point is within the environment boundary
        function result = isInSpaceBoundary(obj, p)
            result = all(p > obj.boundary(:, 1).') && all(p < obj.boundary(:, 2).');
        end
        
        % Generate a random point within the environment boundary
        function P = randomPoint(obj)
            P = (obj.boundary(:, 1) + (obj.boundary(:, 2) - obj.boundary(:, 1)) .* rand(obj.dimension, 1)).';
        end
        
        % Calculate the Euclidean distance to the goal
        function d = distanceToGoal(obj, p)
            d = Environment.Distance(p, obj.goal);
        end
        
        % Get the normalized direction from a point to the goal
        function n = directionToGoal(obj, p)
            n = Environment.Direction(p, obj.goal);
        end
        
        % Check if a point is within the goal threshold
        function success = reachedGoal(obj, p, threshold)
            success = obj.distanceToGoal(p) < threshold;
        end
        
        % Check if a line (from node1 to node2) collides with any obstacles
 % Check if a line (from node1 to node2) or a circle collides with any obstacles
function is_coll = checkCollision(obj, node1, node2_or_circle)
    % Case 1: Line segment collision detection (Original functionality)
    if isvector(node1) && isvector(node2_or_circle)
        if ~obj.isInSpaceBoundary(node1) || ~obj.isInSpaceBoundary(node2_or_circle)
            is_coll = true;  % Out of boundary is considered a collision
            return;
        end
        
        % Check collision only with nearby obstacles
        for i = 1:length(obj.obstacles)
            if obj.obstacles(i).checkLineCollision(node1, node2_or_circle)
                is_coll = true;
                return;
            end
        end
        is_coll = false;
    
    % Case 2: Circle-based collision detection (New functionality)
    elseif isstruct(node2_or_circle)
        % Assuming the input is a struct with fields 'center' and 'radius'
        circle = node2_or_circle;  
        
        % Check if the circle center is within the environment boundary
        if ~obj.isInSpaceBoundary(circle.center)
            is_coll = true;
            return;
        end
        
        % Check collision with all obstacles
        for i = 1:length(obj.obstacles)
            if obj.obstacles(i).checkCircleCollision(circle.center, circle.radius)
                is_coll = true;
                return;
            end
        end
        
        % No collision detected
        is_coll = false;
        
    else
        error('Invalid input type for checkCollision. Provide either two points or a circle struct.');
    end
end




        
        % Plot the environment (boundary, start, goal, obstacles)
        function h = plot(obj)
            obj.axis_handle = axes('Box', 'on', ...
                         'XMinorTick', 'on', 'YMinorTick', 'on', ...   % Minor ticks for better visualization
                         'TickDir', 'in', ...               % Ticks directed inward
                         'XAxisLocation', 'bottom', ...     % Keep bottom tick labels
                         'YAxisLocation', 'left', ...       % Keep left tick labels
                         'TickLength', [0.02 0.02], ...     % Set tick length
                         'XTickLabelMode', 'manual', ...      % Enable auto tick labels on bottom
                         'YTickLabelMode', 'manual');         % Enable auto tick labels on left
            hold all;
            % Turn on the grid and set it to light gray
            grid(obj.axis_handle, 'off');  % Turn on the grid
            set(obj.axis_handle, 'GridColor', [0.8, 0.8, 0.8], ...  % Set grid color to light gray
                'GridAlpha', 1);                 % Set grid transparency (light grid)

            % Ensure grid is applied to both major and minor ticks
            set(obj.axis_handle, 'MinorGridColor', [0.8, 0.8, 0.8], ...  % Light gray for minor grid
                'MinorGridAlpha', 1);                 % Lighter grid for minor ticks
            grid minor;  % Turn on minor grid as well\
            grid off;

            plot(obj.axis_handle,obj.start(1), obj.start(2), 'gs', 'MarkerSize', 10, 'LineWidth', 2); % Start point
            plot(obj.axis_handle,obj.goal(1), obj.goal(2), 'rp', 'MarkerSize', 10, 'LineWidth', 2); % Goal point
           
            % Optionally add the image if `add_image` is true
            if obj.add_image
                img = imread('ADROCO.png'); % Replace with your image file
                img = im2double(img);
                alpha = 0.3; % Change this value to adjust opacity (0.0 - 1.0)
                ax_image = axes('Position', [0.14 0.82 0.4 0.1]); % Adjust position and size as needed
                image('CData', flipud((img)), 'AlphaData', alpha, 'Parent', ax_image);
                axis(ax_image, 'off');
                set(gcf, 'CurrentAxes', obj.axis_handle);
            end

            h = [];
            for i = 1:length(obj.obstacles)
                h(i) = obj.obstacles(i).plot; % Plot each obstacle
            end
            axis_limit = obj.boundary';
            axis(obj.axis_handle, axis_limit(:)');
            
        end
    end
    
    methods(Static)
        % Normalize a vector
        function n = Norm(p)
            n = p / norm(p);
        end
        
        % Get the direction from p1 to p2
        function n = Direction(p1, p2)
            n = Environment.Norm(p2 - p1);
        end
        
        % Calculate the Euclidean distance between two points
        function d = Distance(p1, p2)
            d = sqrt(sum((p1 - p2).^2, 2));
        end
    end
end
