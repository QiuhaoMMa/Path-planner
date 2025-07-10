function environment = AGV_createScene(scene_id, start, goal,add_image_option)
if nargin < 2
    add_image_option = false;
end

obstacles_vertices = generatingObstacles(scene_id);


obstacles = Obstacle.empty;
for i = 1:numel(obstacles_vertices)
    obstacles(i) = Obstacle('polygon', 'vertices', obstacles_vertices{i});
end

boundary = [1 29; 1 29];

environment = Environment(start, goal, boundary, 'obstacles', obstacles, 'resolution', 2, 'add_image', add_image_option);


end

    function obstacles_vertices = generatingObstacles(scene_id)
        obstacles_vertices = [];

        % Predefine number of points for circular shapes
        numPoints = 50;
        switch scene_id

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Macro Obstacles

            case 1
                obstacles_vertices{1} = [7 15 15 7;23 23 13 13];
                obstacles_vertices{2} = [20 25 25 9 9 20;21 21 3 3 9 9];
            case 2
                % Rectangular obstacles_vertices

                obstacles_vertices{1} = [8 15 15 8; 9 9 4 4];
                obstacles_vertices{2} = [5 15 5;22 12 12];
                obstacles_vertices{3} = [14 22 22 19 19 14;26 26 16 16 22 22];
                % Circular block
                obstacles_vertices{4} = generateCircles(22, 9, 5, numPoints);

            case 3
                % Combination of rectangular and circular obstacles_vertices
                obstacles_vertices{1} = [1 11 11 1; 29 29 25 25];
                obstacles_vertices{2} = [4 10 10 4; 13 13 4 4];
                obstacles_vertices{3} = [12 23 23 20 20 12; 27 27 17 17 24 24];
                obstacles_vertices{4} = [13 17 17 13; 19 19 8 8];
                obstacles_vertices{5} = [13 18 18 13; 4 4 1 1];
                obstacles_vertices{6} = generateCircles(7, 19, 3, numPoints);
                obstacles_vertices{7} = generateCircles(25, 9, 4, numPoints);

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Micro Obstacles

            case 4
                % Large set of circles
                centerX = [ 4 7 10 20 26 26];
                centerY = [ 16 21 6 23 15 6];

                % Loop through each center point and generate a circle
                for i = 1:length(centerX)
                    obstacles_vertices{i} = generateCircles(centerX(i), centerY(i), 1, numPoints);
                end

                % Additional larger circles
                obstacles_vertices{7} = generateCircles(15, 15, 1.5, numPoints);

            case 5
                % Another set of circles
                centerX = [4 8 12 16 16 21 21 23 24 26];
                centerY = [16 7 23 27 11 16 8 3 23 15];

                % Loop through each center point and generate a circle
                for i = 1:length(centerX)
                    obstacles_vertices{i} = generateCircles(centerX(i), centerY(i), 1, numPoints);
                end

                % Additional larger circles
                obstacles_vertices{11} = generateCircles(5, 25, 1.5, numPoints);
                obstacles_vertices{12} = generateCircles(10, 17, 1.5, numPoints);
                obstacles_vertices{13} = generateCircles(16, 3, 1.5, numPoints);
                obstacles_vertices{14} = generateCircles(26, 19, 1.5, numPoints);

            case 6
                % Large set of circles
                centerX = [2 3 4 7 8 8 10 11 12 12 13 14 15 16 16 17 19 20 21 21 22 23 24 25 26 26];
                centerY = [21 6 16 12 7 21 13 26 23 10 5 20 15 27 11 6 12 20 16 8 27 3 23 11 15 6];

                % Loop through each center point and generate a circle
                for i = 1:length(centerX)
                    obstacles_vertices{i} = generateCircles(centerX(i), centerY(i), 1, numPoints);
                end

                % Additional larger circles
                obstacles_vertices{27} = generateCircles(4, 11, 1.5, numPoints);
                obstacles_vertices{28} = generateCircles(5, 25, 1.5, numPoints);
                obstacles_vertices{29} = generateCircles(10, 17, 1.5, numPoints);

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Classic Maze

            case 7
                obstacles_vertices{1} = [1 13 13 12 12 1; 12 12 8 8 11 11];
                obstacles_vertices{2} = [8 8 12 12 8 8 7 7; 29 26 26 25 25 19 19 29];
                obstacles_vertices{3} = [15 20 20 19 19 15; 18 18 1 1 17 17];
                obstacles_vertices{4} = [18 29 29 18; 23 23 22 22];
                obstacles_vertices{5} = [25 29 29 25; 12 12 11 11];

            case 8
                obstacles_vertices{1} = [7 7 3 3 7 7 13 13 8 8; 1 8 8 9 9 19 19 18 18 1];
                obstacles_vertices{2} = [10 11 11 17 17 16 16 10;18 18 13 13 8 8 12 12];
                obstacles_vertices{3} = [17 17 19 19 20 20 24 24;9 10 10 17 17 10 10 9];
                obstacles_vertices{4} = [15 16 16 22 22 21 21 13 13 15;16 16 24 24 29 29 25 25 24 24];
                obstacles_vertices{5} = [1 1 7 7 8 8;23 24 24 27 27 23];
                obstacles_vertices{6} = [24 25 25 29 29 24;13 13 15 15 16 16];
                obstacles_vertices{7} = [22 22 28 28 23 23;1 7 7 6 6 1];

            case 9
                obstacles_vertices{1} = [8 9 9 8; 1 1 5 5];
                obstacles_vertices{2} = [13 19 19 22 22 19 19 18 18 13; 3 3 7 7 8 8 11 11 4 4];
                obstacles_vertices{3} = [1 4 4 5 5 12 12 5 5 4 4 1; 8 8 4 4 12 12 13 13 18 18 9 9];
                obstacles_vertices{4} = [8 8 16 16 19 19 13 13 12 12 10 10 15 15; 9 8 8 16 16 17 17 21 21 17 17 16 16 9];
                obstacles_vertices{5} = [25 26 26 23 23 26 26 25 25 23 23 21 21 20 20 17 17 22 22 25; 6 6 11 11 16 16 21 21 17 17 27 27 29 29 27 27 26 26 10 10];
                obstacles_vertices{6} = [12 19 19 13 13 12; 23 23 24 24 29 29];
                obstacles_vertices{7} = [1 1 7 7 4 4 8 8; 26 25 25 20 20 19 19 26];
                obstacles_vertices{8} = [1 4 4 1; 22 22 23 23];
                obstacles_vertices{9} = [19 26 26 25 25 19; 3 3 6 6 4 4];

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Ring Maze
            case 10
                obstacles_vertices{1} = [generateSemiCircles(15, 15, 5*sqrt(2), numPoints,-pi/4+pi/16,7*pi/4-pi/16), fliplr(generateSemiCircles(15, 15, 6*sqrt(2), numPoints,-pi/4+pi/16,7*pi/4-pi/16))];
                obstacles_vertices{2} = [generateSemiCircles(15, 15, 2*sqrt(2), numPoints,-3*pi/2+pi/8,pi/2-pi/8), fliplr(generateSemiCircles(15, 15, 3*sqrt(2), numPoints,-3*pi/2+pi/8,pi/2-pi/8))];

            case 11
                obstacles_vertices{1} = [generateSemiCircles(15, 15, 8*sqrt(2), numPoints,-3*pi/4+pi/20,5*pi/4-pi/20), fliplr(generateSemiCircles(15, 15, 9*sqrt(2), numPoints,-3*pi/4+pi/20,5*pi/4-pi/20))];
                obstacles_vertices{2} = [generateSemiCircles(15, 15, 5*sqrt(2), numPoints,-pi/4+pi/16,7*pi/4-pi/16), fliplr(generateSemiCircles(15, 15, 6*sqrt(2), numPoints,-pi/4+pi/16,7*pi/4-pi/16))];
                obstacles_vertices{3} = [generateSemiCircles(15, 15, 2*sqrt(2), numPoints,-3*pi/2+pi/8,pi/2-pi/8), fliplr(generateSemiCircles(15, 15, 3*sqrt(2), numPoints,-3*pi/2+pi/8,pi/2-pi/8))];

            case 12

                obstacles_vertices{1} = [generateSemiCircles(15, 15, 9*sqrt(2), numPoints,-3*pi/4+pi/20,5*pi/4-pi/20), fliplr(generateSemiCircles(15, 15, 9.9*sqrt(2), numPoints,-3*pi/4+pi/20,5*pi/4-pi/20))];
                obstacles_vertices{2} = [generateSemiCircles(15, 15, 6.5*sqrt(2), numPoints,-pi/4+pi/16,7*pi/4-pi/16), fliplr(generateSemiCircles(15, 15, 7.5*sqrt(2), numPoints,-pi/4+pi/16,7*pi/4-pi/16))];
                obstacles_vertices{3} = [generateSemiCircles(15, 15, 4*sqrt(2), numPoints,-3*pi/2+pi/10,pi/2-pi/10), fliplr(generateSemiCircles(15, 15, 5*sqrt(2), numPoints,-3*pi/2+pi/10,pi/2-pi/10))];
                obstacles_vertices{4} = [generateSemiCircles(15, 15, 1.5*sqrt(2), numPoints,-3*pi/4+pi/6,5*pi/4-pi/6), fliplr(generateSemiCircles(15, 15, 2.3*sqrt(2), numPoints,-3*pi/4+pi/6,5*pi/4-pi/6))];



                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Slim Channel
            case 13
                obstacles_vertices{1} = [1 29 29 17 17 1; 29 29 11 11 21 21];
                obstacles_vertices{2} = [1 15 15 29 29 1;19 19 9 9 1 1];
            case 14
                obstacles_vertices{1} = [1 29 29 18 18 14 14 1;29 29 9 9 16 16 23 23];
                obstacles_vertices{2} = [1 12 12 16 16 29 29 1; 21 21 14 14 7 7 1 1];
            case 15
                obstacles_vertices{1} = [1 29 29 19 19 16 16 13 13 1;29 29 10 10 14 14 18 18 22 22];
                obstacles_vertices{2} = [1 11 11 14 14 17 17 29 29 1;20 20 16 16 12 12 8 8 1 1];

                %%%%%%%%%%%%%%%%%%%%%%%%% Indoor floor map
            case 16
                obstacles_vertices{1} = [1 11 11 1; 18 18 17 17];
                obstacles_vertices{2} = [15 22 22 16 16 15;18 18 17 17 1 1];

            case 17
                obstacles_vertices{1} = [1 6 6 1;11 11 10 10];
                obstacles_vertices{2} = [10 15 15 11 11 10 10; 11 11 10 10 1 1 11];
                obstacles_vertices{3} = [18 24 24 19 19 18;11 11 10 10 1 1];
                obstacles_vertices{4} = [1 13 13 1; 17 17 16 16];
                obstacles_vertices{5} = [17 18 18 25 25 17;29 29 17 17 16 16];

            case 18
                obstacles_vertices{1} = [1 5 5 1; 6 6 5 5];
                obstacles_vertices{2} = [22 23 23 26 26 22;27 27 24 24 23 23];
                obstacles_vertices{3} = [8 12 12 9 9 8;6 6 5 5 1 1];
                obstacles_vertices{4} = [15 19 19 16 16 15; 6 6 5 5 1 1];
                obstacles_vertices{5} = [22 23 23 22; 6 6 1 1];
                obstacles_vertices{6} = [26 29 29 26;6 6 5 5];
                obstacles_vertices{7} = [1 12 12 1; 10 10 9 9];
                obstacles_vertices{8} = [5 26 26 23 23 22 22 16 16 19 19 15 15 5;19 19 18 18 9 9 18 18 10 10 9 9 18 18];
                obstacles_vertices{9} = [1 8 8 1; 24 24 23 23];
                obstacles_vertices{10} = [11 12 12 15 15 11;29 29 24 24 23 23];
                obstacles_vertices{11} = [18 19 19 18;29 29 23 23];


                %%%%%%%%%%%% Serpentine
            case 19

                obstacles_vertices{1} = [9 9 19 19 12 12 21 21 11 11;1 14 14 21 21 23 23 12 12 1 ];
                obstacles_vertices{2} = [16 16 9 9 14 14 12 12 7 7;16 18 18 25 25 29 29 27 27 16];
            case 20
                obstacles_vertices{1} = [6 17 17 8 8 15 15 8 8 15 15 6;7 7 9 9 18 18 20 20 27 27 29 29];
                obstacles_vertices{2} = [12 12 19 19 12 12 19 19 11 11 21 21;25 23 23 15 15 13 13 3 3 1 1 25];
            case 21
                obstacles_vertices{1} = [9 16 16 10 10 16 16 10 10 16 16 10 10 16 16 9;29 29 28 28 21 21 20 20 13 13 12 12 5 5 4 4];
                obstacles_vertices{2} = [13 20 20 19 19 13 13 19 19 13 13 19 19 13;25 25 1 1 8 8 9 9 16 16 17 17 24 24];


                %%%%%%%%%%%%%% Enclosed Starting Point
            case 22
                obstacles_vertices{1} = [1 7 7 2 2 6 6 4 4 7 7 1;8 8 7 7 2 2 4 4 5 5 1 1];
            case 23
                obstacles_vertices{1} = [1 13 13 2 2 12 12 5 5 9 9 7 7 10 10 4 4 13 13 1;14 14 13 13 2 2 10 10 5 5 7 7 8 8 4 4 11 11 1 1];
            case 24
                obstacles_vertices{1} = [1 19 19 2 2 18 18 5 5 15 15 8 8 12 12 10 10 13 13 7 7 16 16 4 4 19 19 1;20 20 19 19 2 2 16 16 5 5 13 13 8 8 10 10 11 11 7 7 14 14 4 4 17 17 1 1];

                %%%%%%%%%%%%%% Enclosed Goal Point
            case 25
                obstacles_vertices{1} = [1 7 7 2 2 6 6 4 4 7 7 1;8 8 7 7 2 2 4 4 5 5 1 1];
            case 26
                obstacles_vertices{1} = [1 13 13 2 2 12 12 5 5 9 9 7 7 10 10 4 4 13 13 1;14 14 13 13 2 2 10 10 5 5 7 7 8 8 4 4 11 11 1 1];
            case 27
                obstacles_vertices{1} = [1 19 19 2 2 18 18 5 5 15 15 8 8 12 12 10 10 13 13 7 7 16 16 4 4 19 19 1;20 20 19 19 2 2 16 16 5 5 13 13 8 8 10 10 11 11 7 7 14 14 4 4 17 17 1 1];
                %%%%%%%%%%%%%%%%%%%%%%%%% Narrow Gate
            case 28
                obstacles_vertices{1} = [10 20 20 10;29 29 17 17];
                obstacles_vertices{2} = [10 20 20 10 ;14 14 1 1];
            case 29
                obstacles_vertices{1} = [10 20 20 10;29 29 17 17];
                obstacles_vertices{2} = [10 20 20 10 ;15 15 1 1];
            case 30
                obstacles_vertices{1} = [10 20 20 10;29 29 16 16];
                obstacles_vertices{2} = [10 20 20 10 ;15 15 1 1];

                %%%%%%%%%%%%%%%%%% Branching Path

            case 31
                obstacles_vertices{1} = [1 6 6 21 26 1; 4 9 24 24 29 29];
                obstacles_vertices{2} = [9 18 9; 12 21 21];
                obstacles_vertices{3} = [4 29 29; 1 26 1];

            case 32
                obstacles_vertices{1} = [1 6 6 21 26 1; 4 9 24 24 29 29];
                obstacles_vertices{2} = [9 18 9; 12 21 21];
                obstacles_vertices{3} = [12 21 21; 9 9 18];
                obstacles_vertices{4} = [4 9 24 24 29 29; 1 6 6 21 26 1];
            case 33
                obstacles_vertices{1} = [1 3 1; 4 6 8];
                obstacles_vertices{2} = [4 8 6; 1 1 3];
                obstacles_vertices{3} = [9 26 26 25 25 9; 4 4 21 21 5 5];
                obstacles_vertices{4} = [8 22 22 8; 8 8 22 22];
                obstacles_vertices{5} = [4 5 5 21 21 4; 9 9 25 25 26 26];
                obstacles_vertices{6} = [26 22 24; 29 29 27];
                obstacles_vertices{7} = [27 29 29; 24 26 22];

        end




        % Helper function for generating circles
        function block = generateCircles(centerX, centerY, radius, numPoints)
            theta = linspace(0, 2*pi, numPoints);  % Generate the range of angles
            x = centerX + radius * cos(theta);     % X coordinates of the circle
            y = centerY + radius * sin(theta);     % Y coordinates of the circle
            block = [x; y];                        % Store the coordinates of the circle
        end



        function block = generateSemiCircles(centerX, centerY, radius, numPoints,rangeStar,rangeEnd)
            theta = linspace(rangeStar, rangeEnd, numPoints); % Generate the range of angles
            x = centerX + radius * cos(theta); % X coordinates of the circle
            y = centerY + radius * sin(theta); % Y coordinates of the circle
            block = [x; y]; % Store the coordinates of the circle
        end
    end



