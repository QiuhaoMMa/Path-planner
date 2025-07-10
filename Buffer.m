classdef Buffer < handle
    properties
        size        % Size of the buffer
        buffer      % The buffer to hold the data
        insert_idx  % Index for the next insertion (circular)
    end
    
    methods
        % Constructor to initialize the buffer with a given size and optional initial value
        function obj = Buffer(size, dim, initialize)
            obj.size = size;
            obj.buffer = zeros(dim, size) + initialize;  % Preallocate buffer with an initial value
            obj.insert_idx = 1;  % Start inserting at the first index
        end
        
        % Method to insert new data into the buffer (circular buffer)
        function insertData(obj, data)
            % Insert data at the current insert index
            obj.buffer(:,obj.insert_idx) = data(:);
            
            % Update the insert index (circular)
            obj.insert_idx = mod(obj.insert_idx, obj.size) + 1;
        end
        
        % Method to retrieve the current state of the buffer in order
        function currentBuffer = getBuffer(obj)
            % Return the buffer starting from the current insertion index
            currentBuffer = [obj.buffer(:, obj.insert_idx:end), obj.buffer(:, 1:obj.insert_idx-1)];
        end

        function P = growingRate(obj)
            decay_factor=0.9;
            % Get the buffer data
            Y = obj.getBuffer();
            m = size(Y, 1);  % Number of rows in Y (data dimensions)
        
            % Time vector for the data points
            X = linspace(1, obj.size, obj.size);
            
            % Create the exponential weights based on the decay_factor
            weights = decay_factor.^(obj.size:-1:1);  % Exponentially decreasing weights
        
            % Initialize the design matrix for linear regression
            X_design = [X; ones(1, obj.size)]';  % Adding a bias (intercept)
        
            P = zeros(2, m);  % To store the slope (growing rate) and intercept for each dimension
            
            % Perform weighted regression for each row of Y
            for i = 1:m
                % Apply weights to X and Y for weighted regression
                W = diag(weights);  % Diagonal weight matrix
                
                % Perform weighted least squares regression: (X'WX)^{-1}X'WY
                X_weighted = W * X_design;
                Y_weighted = W * Y(i, :)';
                
                % Solve for the coefficients using weighted least squares
                beta = (X_weighted' * X_weighted) \ (X_weighted' * Y_weighted);
        
                % Store the slope and intercept for this row
                P(1, i) = beta(1);  % Slope (growing rate)
                P(2, i) = beta(2);  % Intercept
            end
        end

 

        function P=growingEuclidianRate(obj)
            y=obj.getBuffer();
            m=mean(y,2);
            y=y-m;
            y=sqrt(sum(y.^2,1));
            P = mean(y);
        end
    end
end
