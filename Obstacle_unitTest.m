classdef Obstacle_unitTest < matlab.unittest.TestCase
    methods (Test)
        % Test constructor for circle type
        function testCircleConstructor(testCase)
            position = [0, 0];
            radius = 5;
            obs = Obstacle('circle', 'position', position, 'vertices',  radius);
            
            % Validate properties
            testCase.verifyEqual(obs.type, 'circle');
            testCase.verifyEqual(obs.position, position);
            testCase.verifyEqual(obs.vertices, radius);
        end
        
        % Test constructor for bbox type
        function testBboxConstructor(testCase)
            position = [0, 0];
            width = 10;
            height = 20;
            obs = Obstacle('bbox', 'position', position, 'vertices',  [width, height]);
            
            % Validate properties
            testCase.verifyEqual(obs.type, 'bbox');
            testCase.verifyEqual(obs.position, position);
            testCase.verifyEqual(obs.vertices,[   -5     5     5    -5
                -10   -10    10    10]);
        end
        
        % Test point collision for circle
        function testCirclePointCollision(testCase)
            obs = Obstacle('circle','position', [0, 0], 'vertices',5);
            pointInside = [3, 4];  % Inside the circle
            pointOutside = [6, 6]; % Outside the circle
            
            is_coll = obs.checkPointCollision(pointInside);
            testCase.verifyTrue(is_coll, 'Point should be inside the circle.');
            
            is_coll = obs.checkPointCollision(pointOutside);
            testCase.verifyFalse(is_coll, 'Point should be outside the circle.');
        end
        
        
        % Test point collision for bbox
        function testBboxPointCollision(testCase)
            obs = Obstacle('bbox', 'position', [0, 0], 'vertices',[10, 20]);
            pointInside = [3, 4];  % Inside the bbox
            pointOutside = [15, 25]; % Outside the bbox
            
            is_coll = obs.checkPointCollision(pointInside);
            testCase.verifyTrue(is_coll, 'Point should be inside the bbox.');
            
            is_coll = obs.checkPointCollision(pointOutside);
            testCase.verifyFalse(is_coll, 'Point should be outside the bbox.');
        end
        
        % Test line collision for circle
        function testPolygonLineCollision(testCase)
            obs = Obstacle('polygon','vertices', [1 13 13 12 12 1;
                12 12 8 8 11 11]);
            
            obs.plot()
            lineOutside = [0, 8; 10, 10.5];  % Completely outside the circle
            lineIntersect = [0, 8; 6, 11.5];   % Intersects the circle
            
            is_coll = obs.checkLineCollision(lineOutside(1, :), lineOutside(2, :));
            testCase.verifyFalse(is_coll, 'Line should not collide with the circle.');
            
            is_coll = obs.checkLineCollision(lineIntersect(1, :), lineIntersect(2, :));
            testCase.verifyTrue(is_coll, 'Line should collide with the circle.');
        end
        
        % Test line collision for circle
        function testCircleLineCollision(testCase)
            obs = Obstacle('circle', 'position', [0, 0], 'vertices',5);
            lineOutside = [-10, -10; -6, -6];  % Completely outside the circle
            lineIntersect = [-10, 0; 10, 0];   % Intersects the circle
            
            is_coll = obs.checkLineCollision(lineOutside(1, :), lineOutside(2, :));
            testCase.verifyFalse(is_coll, 'Line should not collide with the circle.');
            
            is_coll = obs.checkLineCollision(lineIntersect(1, :), lineIntersect(2, :));
            testCase.verifyTrue(is_coll, 'Line should collide with the circle.');
        end
        
        % Test line collision for bbox
        function testBboxLineCollision(testCase)
            obs = Obstacle('bbox', 'position', [0, 0], 'vertices',[1, 4]);
            
            obs.plot
            lineOutside = [-15, -15; -10, -10];  % Completely outside the bbox
            lineIntersect = [0, 2; 10, 2];     % Intersects the bbox
            
            is_coll = obs.checkLineCollision(lineOutside(1, :), lineOutside(2, :));
            testCase.verifyFalse(is_coll, 'Line should not collide with the bbox.');
            
            is_coll = obs.checkLineCollision(lineIntersect(1, :), lineIntersect(2, :));
            testCase.verifyTrue(is_coll, 'Line should collide with the bbox.');
        end
    end
end
