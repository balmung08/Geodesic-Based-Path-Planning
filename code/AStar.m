function [path_x, path_y] = AStar(start, goal, rect_obstacles, circle_obstacles, inflate_radius, grid_resolution, map_bounds, num_points)
    % 输入参数:
    %   start: 起点坐标 [x, y]
    %   goal: 终点坐标 [x, y]
    %   rect_obstacles: 矩形障碍物矩阵，每行定义为一个矩形 [x1, y1, x2, y2]
    %   circle_obstacles: 圆形障碍物矩阵，每行定义为 [cx, cy, radius]
    %   inflate_radius: 障碍物膨胀半径
    %   grid_resolution: 网格分辨率
    %   map_bounds: 地图边界 [xmin, xmax, ymin, ymax]
    %   num_points: 重采样后的路径点数 (可选)
    %
    % 输出:
    %   path_x: 路径点的x坐标
    %   path_y: 路径点的y坐标

    % 创建网格
    x_grid = map_bounds(1):grid_resolution:map_bounds(2);
    y_grid = map_bounds(3):grid_resolution:map_bounds(4);
    [nI, nJ] = deal(length(x_grid), length(y_grid));
    
    % 初始化障碍物地图 (0=自由, 1=障碍)
    obs_map = zeros(nI, nJ);
    
    % 改进的矩形障碍物膨胀（圆形角点）
    for k = 1:size(rect_obstacles, 1)
        rect = rect_obstacles(k, :);
        x_min = min(rect(1), rect(3));
        x_max = max(rect(1), rect(3));
        y_min = min(rect(2), rect(4));
        y_max = max(rect(2), rect(4));
        
        for i = 1:nI
            for j = 1:nJ
                x = x_grid(i);
                y = y_grid(j);
                
                % 计算点到矩形的最短距离
                dist_to_rect = point_to_rectangle_distance(x, y, x_min, x_max, y_min, y_max);
                
                % 如果距离小于等于膨胀半径，则标记为障碍物
                if dist_to_rect <= inflate_radius
                    obs_map(i, j) = 1;
                end
            end
        end
    end
    
    % 膨胀并标记圆形障碍物
    for k = 1:size(circle_obstacles, 1)
        circle = circle_obstacles(k, :);
        cx = circle(1);
        cy = circle(2);
        r = circle(3) + inflate_radius;
        
        for i = 1:nI
            for j = 1:nJ
                x = x_grid(i);
                y = y_grid(j);
                if (x - cx)^2 + (y - cy)^2 <= r^2
                    obs_map(i, j) = 1;
                end
            end
        end
    end
    
    % 转换起点/终点到网格索引
    [~, start_i] = min(abs(x_grid - start(1)));
    [~, start_j] = min(abs(y_grid - start(2)));
    [~, goal_i] = min(abs(x_grid - goal(1)));
    [~, goal_j] = min(abs(y_grid - goal(2)));
    
    % 检查起点/终点是否在障碍物内
    if obs_map(start_i, start_j)
        error('起点位于障碍物内！');
    end
    if obs_map(goal_i, goal_j)
        error('终点位于障碍物内！');
    end
    
    % 初始化A*算法数据结构
    g_score = inf(nI, nJ);
    parent_i = zeros(nI, nJ);
    parent_j = zeros(nI, nJ);
    closed_set = false(nI, nJ);
    open_list = [];
    
    % 添加起点到开放列表
    g_score(start_i, start_j) = 0;
    f_start = g_score(start_i, start_j) + heuristic([x_grid(start_i), y_grid(start_j)], goal);
    open_list = [open_list; start_i, start_j, f_start];
    
    % 8个移动方向 (上,下,左,右,对角线)
    directions = [1,0; -1,0; 0,1; 0,-1; 1,1; 1,-1; -1,1; -1,-1];
    
    % A*主循环
    while ~isempty(open_list)
        % 寻找开放列表中f值最小的节点
        [~, idx] = min(open_list(:,3));
        current = open_list(idx, :);
        open_list(idx, :) = [];
        
        i = current(1); j = current(2);
        
        % 如果当前节点已关闭，跳过
        if closed_set(i, j)
            continue;
        end
        closed_set(i, j) = true;
        
        % 检查是否到达目标
        if i == goal_i && j == goal_j
            [path_x, path_y] = reconstruct_path(parent_i, parent_j, i, j, x_grid, y_grid);
            
            % 如果指定了重采样点数，则进行重采样
            if nargin > 7 && num_points > 0
                [path_x, path_y] = resample_path(path_x, path_y, num_points);
            end
            return;
        end
        
        % 探索邻居节点
        for k = 1:size(directions, 1)
            ni = i + directions(k, 1);
            nj = j + directions(k, 2);
            
            % 跳过无效邻居
            if ni < 1 || ni > nI || nj < 1 || nj > nJ || obs_map(ni, nj) || closed_set(ni, nj)
                continue;
            end
            
            % 计算移动代价
            if abs(directions(k,1)) + abs(directions(k,2)) == 2
                move_cost = grid_resolution * sqrt(2); % 对角线移动
            else
                move_cost = grid_resolution; % 直线移动
            end
            
            tentative_g = g_score(i, j) + move_cost;
            
            % 找到更优路径
            if tentative_g < g_score(ni, nj)
                g_score(ni, nj) = tentative_g;
                f_score = tentative_g + heuristic([x_grid(ni), y_grid(nj)], goal);
                parent_i(ni, nj) = i;
                parent_j(ni, nj) = j;
                open_list = [open_list; ni, nj, f_score];
            end
        end
    end
    
    % 路径未找到
    path_x = [];
    path_y = [];
    warning('未找到可行路径！');
end

% --- 辅助函数 ---
function h = heuristic(a, b)
    % 欧几里得距离启发函数
    h = norm(a - b);
end

function [path_x, path_y] = reconstruct_path(parent_i, parent_j, end_i, end_j, x_grid, y_grid)
    % 回溯重建路径
    path_x = [];
    path_y = [];
    i = end_i;
    j = end_j;
    
    while parent_i(i, j) ~= 0 || parent_j(i, j) ~= 0
        path_x = [x_grid(i), path_x];
        path_y = [y_grid(j), path_y];
        i_temp = parent_i(i, j);
        j_temp = parent_j(i, j);
        i = i_temp;
        j = j_temp;
    end
    path_x = [x_grid(i), path_x];
    path_y = [y_grid(j), path_y];
end

function [resampled_x, resampled_y] = resample_path(x, y, n_points)
    % 路径重采样函数
    % 输入:
    %   x, y: 原始路径点坐标
    %   n_points: 重采样后的点数
    % 输出:
    %   resampled_x, resampled_y: 重采样后的路径点
    
    % 计算累积路径长度
    distances = sqrt(diff(x).^2 + diff(y).^2);
    cumulative_dist = [0, cumsum(distances)];
    total_length = cumulative_dist(end);
    
    % 生成等间距采样点
    sample_distances = linspace(0, total_length, n_points);
    
    % 线性插值
    resampled_x = interp1(cumulative_dist, x, sample_distances, 'linear');
    resampled_y = interp1(cumulative_dist, y, sample_distances, 'linear');
end

function dist = point_to_rectangle_distance(px, py, x_min, x_max, y_min, y_max)
    % 计算点到矩形的最短距离
    % 输入:
    %   px, py: 点的坐标
    %   x_min, x_max, y_min, y_max: 矩形的边界
    % 输出:
    %   dist: 最短距离
    
    % 如果点在矩形内部，距离为0
    if px >= x_min && px <= x_max && py >= y_min && py <= y_max
        dist = 0;
        return;
    end
    
    % 计算点到矩形各边的距离
    dx = max(0, max(x_min - px, px - x_max));
    dy = max(0, max(y_min - py, py - y_max));
    
    % 返回欧几里得距离
    dist = sqrt(dx^2 + dy^2);
end