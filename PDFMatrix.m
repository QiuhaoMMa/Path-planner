classdef PDFMatrix <handle
    properties
        matrix         % PDF matrix
        matrix_size    % Size of the matrix
        fade_func=@(x,r,c) 1./(1+exp(r * (x-c)));  % the fade function generator
        resolution     % Resolution of the matrix for each dimension
        boundary       % Boundary of the space in each dimension
        base_radius=100    % Base radius for neighborhood influence
        dimension      % Number of dimensions
        direction_scale=1
        plot_handle    % Handle for the plot

        MAX            % Maximum value in the matrix
        MIN            % Minimum value in the matrix
    end

    methods
        % Constructor to initialize the matrix based on space bounds and resolution
        function obj = PDFMatrix(varargin)
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                if isprop(obj, key)
                    obj.(key) = value;
                end
            end

            % Calculate matrix dimensions based on space bounds and resolution
            obj.dimension = size(obj.boundary,1);

            % Calculate matrix dimensions based on space bounds and resolution
            obj.matrix_size = ceil((obj.boundary(:, 2) - obj.boundary(:, 1)) ./ obj.resolution(:)).';

            % Initialize matrix with ones based on calculated dimensions
            obj.matrix = zeros(obj.matrix_size);
        end


        % Method to get a normalized direction sample from the matrix
        function normalized_direction = getSample(obj, cutOff)
            % Flatten the matrix and normalize the PDF
            pdf = (obj.matrix(:) - obj.MIN) / (obj.MAX - obj.MIN);

            % Find points exceeding the cutoff
            valid_points = pdf >= 0.99 * cutOff;
            if ~any(valid_points)
                % Random direction if no valid points
                normalized_direction = randn(1, obj.dimension);
                normalized_direction = normalized_direction / norm(normalized_direction);
                return;
            end

            % Get valid indices and extract their coordinates
            linear_idx = find(valid_points);
            coords = obj.index2indices(linear_idx);

            % Compute centroid and normalize the direction
            centroid_vector = mean(coords, 1) - (obj.matrix_size / 2);
            norm_val = norm(centroid_vector);
            normalized_direction = centroid_vector / (norm_val + eps);
        end


        function obj = addPositionSample(obj, position, reward_pos, fade_rate)

            % Normalize the reward_pos value
            reward_pos = reward_pos / 0.5;

            % Compute common elements: offsets and neighbors
            [neighbor_offsets, distances, max_distance, ~] = computeNeighborOffsets(obj, position, []);


            % Compute the influence for all neighbors at once
            influence_factors = reward_pos * obj.fade_func(distances / max_distance, fade_rate, 0);

            % Update the matrix with the combined influence
            updateMatrix(obj, neighbor_offsets,  influence_factors);
        end


        function obj = addDirectionSample(obj,  direction, reward_dir,fade_rate)
            direction = obj.normalizeDirection(direction);

            % Compute neighbor offsets for the given direction
            [neighbor_offsets, ~, ~, point_offset] = obj.computeNeighborOffsets([0, 0], direction);

            Ds = neighbor_offsets - point_offset;
            Ns = Ds ./ vecnorm(Ds, 2, 2);
            Ns(isnan(Ns)) = 0;  % Handle NaNs from zero distances


            % Align direction and compute influence
            aligned = -Ns* direction';
            d = aligned;
            c=0;
            if (reward_dir<-9.5)
                reward_dir=reward_dir/10;
                c=-reward_dir.*(d>0)/2;
                d = min(aligned, 0);
            end
            influence_vel = reward_dir * obj.fade_func(sign(d).*abs(d).^fade_rate, fade_rate, 0)+c;

            % Update the matrix
            updateMatrix(obj, neighbor_offsets,  influence_vel);
        end


        % Method to update the PDF matrix with real-world coordinates and direction-based direction
        function obj = addSample(obj,  position, reward_pos, direction, reward_dir,fade_rate)

            % Normalize reward values
            reward_dir = reward_dir / 0.9;

            % Compute common elements: offsets and neighbors
            [neighbor_offsets, distances, max_distance, point_offset] = computeNeighborOffsets(obj, position, direction);

            % Compute the influence for the position part
            influence_pos = reward_pos * obj.fade_func(distances / max_distance, fade_rate, 0);

            % If direction is specified, compute the influence for the direction part

            direction = obj.normalizeDirection(direction);
            Ds=(neighbor_offsets - point_offset);
            Ns=Ds./vecnorm(Ds,2,2);
            Ns(isnan(Ns))=0;

            aligned = -Ns* direction';
            d = min(aligned, 0) .* (1 - distances / max_distance);
            influence_vel = reward_dir * obj.fade_func(-d .^ fade_rate, fade_rate, 0);


            % Update the matrix with the combined influence
            updateMatrix(obj, neighbor_offsets, influence_pos + influence_vel);
        end

        % Shared function to update the matrix using linear indices
        function updateMatrix(obj, neighbor_offsets, influence)
            % Convert neighbor offsets to linear indices
            neighbor_offsets_cell = num2cell(neighbor_offsets, 1);
            linear_indices = sub2ind(obj.matrix_size, neighbor_offsets_cell{:});

            % Update the matrix values
            obj.matrix(linear_indices) = obj.matrix(linear_indices) + influence;

            obj.MAX=max(obj.matrix(:));
            obj.MIN=min(obj.matrix(:));
        end

        % Normalize direction vector
        function direction = normalizeDirection(obj, vector)
            direction = vector ./ obj.resolution;
            direction = direction / (norm(direction) + eps);  % Prevent division by zero
        end

        % Compute neighbor offsets and distances for the given position and direction
        function [neighbor_offsets, distances, max_distance, point_offset] = computeNeighborOffsets(obj, position, direction)
            % Convert the position to matrix indices
            point_offset = obj.position2indces(position);

            % Determine the radius of influence based on direction
            if isempty(direction)
                affected_steps = obj.base_radius + ceil(obj.direction_scale);
            else
                speed = norm(direction ./ obj.resolution);
                affected_steps = obj.base_radius + ceil(speed * obj.direction_scale);
            end

            % Get the neighbor offsets and filter invalid ones
            neighbor_absolute_offsets = obj.get_neighbor_offsets(affected_steps);
            neighbor_offsets = neighbor_absolute_offsets + point_offset;
            remove_index = any(neighbor_offsets < 1, 2) | any(neighbor_offsets > obj.matrix_size, 2);
            neighbor_offsets(remove_index, :) = [];

            % Compute distances and maximum distance
            distances = vecnorm(neighbor_offsets - point_offset, 2, 2);
            max_distance = max(distances);
        end

        function normalize(obj)
            obj.matrix = obj.matrix / max(abs(obj.matrix(:)));  % Normalize the entire matrix
        end

        function value = getValueByIndex(obj, index)
            value = obj.matrix(index);  % Retrieve the value directly by index
        end

        function value = getNormalizedValueByIndex(obj, index)
            value = (obj.matrix(index) - obj.MIN) / (obj.MAX - obj.MIN);  % Normalize the value directly
        end

        function biasedRandomized(obj, percentage)
            N = numel(obj.matrix);
            M = 1 + floor(N * percentage);
            indx = biasedRandomSelection(1:N, M, 0.01, 1);  % Perform biased random selection
            obj.matrix(indx) = (0.5 - rand(1, M)) * 1e-5;  % Update selected matrix elements
        end

        function value = getValueByPosition(obj, position)
            index = obj.indices2index(obj.position2indces(position));  % Convert position to matrix index
            value = obj.matrix(index);  % Get the value by matrix index
        end

        function value = getNormalizedValueByPosition(obj, position)
            value = (obj.getValueByPosition(position) - obj.MIN) / (obj.MAX - obj.MIN);  % Normalize the value
        end


        function indices = position2indces(obj, position)
            position = max(min(position, obj.boundary(:, 2)'), obj.boundary(:, 1)');  % Bound the position
            indices = ceil((position - obj.boundary(:, 1)') ./ obj.resolution);  % Convert to indices
        end

        function position = indices2position(obj, indices)
            if any(indices < 1) || any(indices > obj.matrix_size)
                return;  % Return early if indices are out of bounds
            end
            position = (indices - 0.5) .* obj.resolution + obj.boundary(:, 1)';  % Convert indices to position
        end

        function indices = index2indices(obj, index)
            subscripts = cell(1, numel(obj.matrix_size));  % Preallocate subscripts
            [subscripts{:}] = ind2sub(obj.matrix_size, index);  % Convert linear index to subscripts
            indices = cell2mat(subscripts);  % Convert to matrix format
        end

        function index = indices2index(obj, indices)
            indices = min(max(indices, 1), obj.matrix_size);  % Ensure indices are within bounds
            subscripts = num2cell(indices, 1);
            index = sub2ind(obj.matrix_size, subscripts{:});  % Convert to linear index
        end

        function position = index2position(obj, index)
            position = obj.indices2position(obj.index2indices(index));  % Convert index to position
        end

        function offsets = get_neighbor_offsets(obj, affected_steps)
            affected_steps = min(affected_steps, max(obj.matrix_size) + 1);  % Limit affected steps to matrix bounds
            base_offsets = arrayfun(@(dim) -affected_steps:affected_steps, 1:obj.dimension, 'UniformOutput', false);
            [grid{1:obj.dimension}] = ndgrid(base_offsets{:});
            offsets = reshape(cat(obj.dimension + 1, grid{:}), [], obj.dimension);  % Reshape into offset matrix
        end

        % Method to plot the PDF matrix (works for 2D and 3D cases)
        function plot(obj,varargin)
            scale=0.25;
            if nargin>1
                scale=varargin{1};
                if scale>0.8
                    scale=0.8;
                end
            end
            if length(obj.matrix_size) == 2
                % Create grid of x, y coordinates scaled between 0 and 1
                x=linspace(obj.boundary(1,1), obj.boundary(1,2), size(obj.matrix, 1));
                y=linspace(obj.boundary(2,1), obj.boundary(2,2), size(obj.matrix, 1));

                [X, Y] = meshgrid(x,y);

                % Transpose the matrix to correct X and Y axes orientation
                % Inset axis: a smaller plot inside the main axis
                inset_ax = axes('Position', [0.0 0.00 scale scale]);  % Specify the inset position and size
                obj.plot_handle=pcolor(inset_ax, X*scale, Y*scale, obj.matrix');  % Plot using pcolor
                shading flat;               % Remove grid lines for a cleaner plot


                % colorbar;
                title('2D PDF Matrix');
                xlabel('X-axis (Real X)');
                ylabel('Y-axis (Real Y)');
                axis equal;

                % axis_limit = obj.boundary'*scale;
                % axis(axis_limit(:)');

            elseif length(obj.matrix_size) == 3
                % 3D plot using slice visualization
                [x, y, z] = ndgrid(1:obj.matrix_size(1), 1:obj.matrix_size(2), 1:obj.matrix_size(3));
                xslice = round(obj.matrix_size(1)/2); % Slice in the middle along X-axis
                yslice = round(obj.matrix_size(2)/2); % Slice in the middle along Y-axis
                zslice = round(obj.matrix_size(3)/2); % Slice in the middle along Z-axis
                slice(x, y, z, obj.matrix, xslice, yslice, zslice);
                colorbar;
                title('3D PDF Matrix');
                xlabel('X-axis');
                ylabel('Y-axis');
                zlabel('Z-axis');
                axis tight;
                shading interp;
            else
                error('Plotting is only supported for 2D and 3D matrices.');
            end
        end
    end
end
