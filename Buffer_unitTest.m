classdef TestBuffer < matlab.unittest.TestCase
    methods (Test)
        function testInitialization1D(testCase)
            % Test buffer initialization with 1D data
            buf = Buffer(5, 1, 0);
            actualBuffer = buf.getBuffer();
            expectedBuffer = zeros(1, 5);  % Expect a 1x5 buffer initialized with 0
            
            % Verify that the buffer is initialized correctly
            testCase.verifyEqual(actualBuffer, expectedBuffer, 'Buffer initialization failed for 1D');
        end
        
        function testInsertion1D(testCase)
            % Test insertion of 1D data into the buffer
            buf = Buffer(5, 1, 0);
            buf.insertData(1);
            buf.insertData(2);
            buf.insertData(3);
            
            % Check the state of the buffer after insertion
            actualBuffer = buf.getBuffer();
            expectedBuffer = [0, 0, 1, 2, 3];  % Expect first three values inserted in a 1D buffer
            
            % Verify that the insertion works correctly
            testCase.verifyEqual(actualBuffer, expectedBuffer, 'Data insertion failed for 1D');
        end
        
        function testCircularInsertion1D(testCase)
            % Test circular insertion behavior when buffer is full (1D)
            buf = Buffer(5, 1, 0);
            buf.insertData(1);
            buf.insertData(2);
            buf.insertData(3);
            buf.insertData(4);
            buf.insertData(5);
            buf.insertData(6);  % This should overwrite the first element (1)
            
            % Check the state of the buffer after circular insertion
            actualBuffer = buf.getBuffer();
            expectedBuffer = [2, 3, 4, 5, 6];  % Expect the first element to be overwritten
            
            % Verify that the circular insertion works as expected
            testCase.verifyEqual(actualBuffer, expectedBuffer, 'Circular insertion failed for 1D');
        end

        function testGrowRate1D(testCase)
            % Test grow rate calculation for 1D data
            buf = Buffer(5, 1, 1);
            buf.insertData(2);
            buf.insertData(3);
            buf.insertData(4);
            buf.insertData(5);
            buf.insertData(6);
            
            % Get the grow rate from the buffer
            actualGrowRate = buf.growingRate();
            
            % Check if the result is a valid grow rate (should return a 1x2 vector for linear fit)
            testCase.verifySize(actualGrowRate, [1 2], 'Grow rate calculation failed for 1D');
        end

        function testInitialization2D(testCase)
            % Test buffer initialization with 2D data
            buf = Buffer(5, 2, 0);  % 2D buffer initialized with 0
            actualBuffer = buf.getBuffer();
            expectedBuffer = zeros(2, 5);  % Expect a 2x5 buffer initialized with 0
            
            % Verify that the buffer is initialized correctly
            testCase.verifyEqual(actualBuffer, expectedBuffer, 'Buffer initialization failed for 2D');
        end

        function testInsertion2D(testCase)
            % Test insertion of 2D data into the buffer
            buf = Buffer(5, 2, 0);  % Create a 2D buffer
            buf.insertData([1; 2]);  % Insert a 2D column vector [1; 2]
            buf.insertData([3; 4]);
            buf.insertData([5; 6]);
            
            % Check the state of the buffer after insertion
            actualBuffer = buf.getBuffer();
            expectedBuffer = [0 0 1 3 5; 0 0 2 4 6];  % Expect first three values inserted in a 2D buffer
            
            % Verify that the insertion works correctly
            testCase.verifyEqual(actualBuffer, expectedBuffer, 'Data insertion failed for 2D');
        end

        function testCircularInsertion2D(testCase)
            % Test circular insertion behavior when buffer is full (2D)
            buf = Buffer(5, 2, 0);  % Create a 2D buffer
            buf.insertData([1; 2]);
            buf.insertData([3; 4]);
            buf.insertData([5; 6]);
            buf.insertData([7; 8]);
            buf.insertData([9; 10]);
            buf.insertData([11; 12]);  % This should overwrite the first element
            
            % Check the state of the buffer after circular insertion
            actualBuffer = buf.getBuffer();
            expectedBuffer = [3 5 7 9 11; 4 6 8 10 12];  % Expect the first element to be overwritten
            
            % Verify that the circular insertion works as expected
            testCase.verifyEqual(actualBuffer, expectedBuffer, 'Circular insertion failed for 2D');
        end
        
        function testGrowRate2D(testCase)
            % Test grow rate calculation for 2D data
            buf = Buffer(5, 2, 1);  % Initialize a 2D buffer
            buf.insertData([2; 3]);
            buf.insertData([3; 4]);
            buf.insertData([4; 5]);
            buf.insertData([5; 6]);
            buf.insertData([6; 7]);
            
            % Get the grow rate from the buffer (should work for each dimension separately)
            actualGrowRate = buf.growingRate();
            
            % Check if the result is a valid grow rate (should return a 2x2 matrix for linear fit)
            testCase.verifySize(actualGrowRate, [2 2], 'Grow rate calculation failed for 2D');
        end
    end
end
