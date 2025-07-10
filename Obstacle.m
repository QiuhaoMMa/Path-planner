classdef Obstacle
    properties
        type        % Type of obstacle: 'circle', 'bbox', or 'polygon'
        position=[0 0]    % Center position for both types [x, y]
        vertices=[] % Vertices for polygon type or bounding box
    end

    methods
        function obj = Obstacle(type, varargin)
            % Constructor to initialize the obstacle
            obj.type = type;
            % Parse optional key-value arguments
            for i = 1:2:length(varargin)
                key = varargin{i};
                value = varargin{i+1};
                if isprop(obj, key)
                    obj.(key) = value;
                end
            end

            switch numel(obj.vertices)
                case 1
                    if ~strcmp(obj.type,'circle')
                        error('Invalid number of vertices for the obstacle type');
                    end
                case 2
                    if ~strcmp(obj.type,'bbox')
                        error('Invalid number of vertices for the obstacle type');
                    end
                    obj.vertices = obj.precomputeBboxVertices(obj.vertices(1), obj.vertices(2));
                otherwise
            end
        end

        function vertices = precomputeBboxVertices(obj,width,height)
            % Precompute bounding box vertices based on position, width, and height
            x_min = obj.position(1) - width / 2;
            x_max = obj.position(1) + width / 2;
            y_min = obj.position(2) - height / 2;
            y_max = obj.position(2) + height / 2;

            vertices = [x_min, y_min;  % Bottom-left
                x_max, y_min;  % Bottom-right
                x_max, y_max;  % Top-right
                x_min, y_max]'; % Top-left
        end

        function is_coll = checkPointCollision(obj, point)
            % Method to check if a point is in collision with the obstacle
            switch obj.type
                case 'circle'
                    is_coll = obj.checkCirclePointCollision(point);
                case 'bbox'
                    is_coll = obj.checkBboxPointCollision(point);
                case 'polygon'
                    is_coll = obj.checkPolygonPointCollision(point);
            end
        end


        function is_coll = checkLineCollision(obj, p1, p2)
            % Method to check if a line is in collision with the obstacle
            switch obj.type
                case 'circle'
                    is_coll = obj.checkCircleLineCollision(p1, p2);
                case 'bbox'
                    is_coll = obj.checkPolygonLineCollision(p1, p2);
                case 'polygon'
                    is_coll = obj.checkPolygonLineCollision(p1, p2);
            end
        end

        function is_coll = checkCircleCollision(obj, circle_center, circle_radius)
            % 检查一个圆形是否与当前障碍物发生碰撞

            switch obj.type
                case 'circle'
                    % 圆 - 圆碰撞检测
                    distance = norm(circle_center - obj.position);
                    is_coll = distance <= (obj.vertices(1) + circle_radius);  % 检查两圆心的距离是否小于等于两半径之和

                case 'bbox'
                    % 圆 - 矩形碰撞检测 (利用 最近点算法)
                    is_coll = obj.checkCircleBBoxCollision(circle_center, circle_radius, obj.vertices);

                case 'polygon'
                    % 圆 - 多边形碰撞检测
                    is_coll = obj.checkCirclePolygonCollision(circle_center, circle_radius, obj.vertices);

                otherwise
                    error('Unknown obstacle type: %s', obj.type);
            end
        end



        function is_coll = checkCircleBBoxCollision(circle_center, circle_radius, bbox_vertices)
            % 检测圆形与矩形 (BBox) 的碰撞
            % bbox_vertices 是一个 2x4 矩阵，每列表示一个顶点

            % 找到矩形的最近点到圆心的距离
            x_min = min(bbox_vertices(1, :));
            x_max = max(bbox_vertices(1, :));
            y_min = min(bbox_vertices(2, :));
            y_max = max(bbox_vertices(2, :));

            closest_x = max(x_min, min(circle_center(1), x_max));
            closest_y = max(y_min, min(circle_center(2), y_max));
            closest_point = [closest_x, closest_y];

            % 检查最近点与圆心的距离
            distance = norm(closest_point - circle_center);
            is_coll = distance <= circle_radius;
        end


        function is_coll = checkCirclePolygonCollision(obj, circle_center, circle_radius, polygon_vertices)
            % 检测圆形与任意多边形的碰撞 (利用 最近点算法 和 inpolygon 函数)
            numVertices = size(polygon_vertices, 2);
            is_coll = false;

            % 检查圆心与多边形的任意边的碰撞
            for i = 1:numVertices
                % 获取多边形的两点形成的边
                p1 = polygon_vertices(:, i);
                if i == numVertices
                    p2 = polygon_vertices(:, 1);  % 封闭多边形
                else
                    p2 = polygon_vertices(:, i+1);
                end

                % 计算从圆心到多边形边的最短距离
                d = obj.pointLineDistance(p1', p2', circle_center);  % 注意这里用的是 obj.pointLineDistance

                % 如果最短距离小于半径，则认为有碰撞
                if d <= circle_radius
                    is_coll = true;
                    return;
                end
            end

            % 检查圆心是否在多边形内部 (使用 MATLAB 自带函数)
            if inpolygon(circle_center(1), circle_center(2), polygon_vertices(1, :), polygon_vertices(2, :))
                is_coll = true;
            end
        end






        function h = plot(obj, varargin)
            % Method to plot the obstacle
            hold on;
            if isempty(varargin)
                varargin = {'EdgeColor', 'k'};
            else
                varargin = [{'EdgeColor', 'k'}, varargin{:}];
            end

            switch obj.type
                case 'circle'
                    h = viscircles(obj.position, obj.radius, varargin{:});
                case 'bbox'
                    h = fill(obj.vertices(1, :), obj.vertices(2, :), [237, 177, 32]/255, varargin{:});
                case 'polygon'
                    h = fill(obj.vertices(1, :), obj.vertices(2, :), [237, 177, 32]/255, varargin{:});
            end
        end

        function is_coll = checkCirclePointCollision(obj, point)
            % Collision check for circle
            distance = norm(point - obj.position);
            is_coll = distance <= obj.vertices(1);
        end

        function is_coll = checkBboxPointCollision(obj, point)
            % Collision check for bounding box
            is_coll = (sum(all(obj.vertices<point',1)+all(obj.vertices>point',1))==2);
        end

        function is_coll = checkPolygonPointCollision(obj, point)
            % Collision check for a polygon (point-in-polygon test)
            is_coll = inpolygon(point(1), point(2), obj.vertices(:,1), obj.vertices(:,2));
        end


        function collision = checkPolygonLineCollision(obj, p1, p2)
            % Check if a line segment (p1-p2) intersects any edge of a polygon
            numVertices = size(obj.vertices, 2);  % Number of vertices in the polygon
            collision = false;

            % Loop through each edge of the polygon
            for i = 1:numVertices
                p3 = obj.vertices(:, i);
                if i == numVertices
                    p4 = obj.vertices(:, 1); % Close the polygon
                else
                    p4 = obj.vertices(:, i+1);
                end

                % Check if the line segment (p1-p2) intersects with edge (p3-p4)
                if obj.checkLineIntersection(p1, p2, p3', p4')
                    collision = true;
                    return;
                end
            end
        end

        function collision = checkCircleLineCollision(obj, p1, p2)
            % Check if a line intersects a circle
            d = obj.pointLineDistance(p1, p2, obj.position);
            collision = d <= obj.vertices(1);
        end

        function intersection = checkLineIntersection(obj, p1, p2, p3, p4)
            % Check if two line segments (p1-p2 and p3-p4) intersect

            % Precompute line equation coefficients for both segments
            A1 = p2(2) - p1(2);  % y2 - y1
            B1 = p1(1) - p2(1);  % x1 - x2
            C1 = A1 * p1(1) + B1 * p1(2);  % Line 1: A1*x + B1*y = C1

            A2 = p4(2) - p3(2);  % y4 - y3
            B2 = p3(1) - p4(1);  % x3 - x4
            C2 = A2 * p3(1) + B2 * p3(2);  % Line 2: A2*x + B2*y = C2

            % Determinant (denominator in intersection formulas)
            det = A1 * B2 - A2 * B1;

            % Early exit for parallel lines
            if abs(det) < 1e-9  % Adjust tolerance for numerical stability
                intersection = false;
                return;
            end

            % Precompute parts of intersection coordinates
            invDet = 1 / det;  % Store inverse of determinant to avoid division twice
            x = (B2 * C1 - B1 * C2) * invDet;
            y = (A1 * C2 - A2 * C1) * invDet;

            % Check if the intersection point (x, y) lies on both line segments
            if obj.checkPointOnSegment([x, y], p1, p2) && obj.checkPointOnSegment([x, y], p3, p4)
                intersection = true;
            else
                intersection = false;
            end
        end




        function onSegment = checkPointOnSegment(~, p, p1, p2)
            % Check if a point p is on the line segment between p1 and p2 in n dimensions
            tolerance = 1e-9;  % Tolerance for floating-point comparisons

            % Vector from p1 to p (v1) and p1 to p2 (v2)
            v1 = p - p1;
            v2 = p2 - p1;

            % Check if the point is within the bounds of the segment in all dimensions
            inBounds = all(p >= min(p1, p2) - tolerance) && all(p <= max(p1, p2) + tolerance);

            % Check for collinearity using cross product for 2D/3D, or dot product for higher dimensions
            if norm(v2) < tolerance  % p1 and p2 are the same point
                collinear = norm(v1) < tolerance;  % Check if p is the same as p1
            else
                % Check for collinearity using dot product
                collinear = abs(dot(v1, v2)) / (norm(v1) * norm(v2)) - 1 < tolerance;  % Ensure the vectors are collinear
            end

            % The point is on the segment if it's collinear and within bounds
            onSegment = inBounds && collinear;
        end



        function d = pointLineDistance(~, p1, p2, p)
            % Compute the shortest distance from point p to the line segment p1-p2
            v = p2 - p1;
            w = p - p1;
            c1 = dot(w, v);
            if c1 <= 0
                d = norm(p - p1);
            else
                c2 = dot(v, v);
                if c2 <= c1
                    d = norm(p - p2);
                else
                    b = c1 / c2;
                    pb = p1 + b * v;
                    d = norm(p - pb);
                end
            end
        end
    end
end
