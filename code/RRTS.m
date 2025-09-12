function [path_x, path_y] = RRTS(start, goal, rect_obstacles, circle_obstacles, inflate_radius, max_iter, step_size, search_radius, goal_bias, map_bounds, num_points, early_stop_patience)
    % 持续优化版本的 RRT* (含早停机制)
    % 新增参数：
    % early_stop_patience - 早停耐心值，连续多少次迭代路径未缩短则停止
    
    if nargin < 11
        num_points = 0;
    end
    
    if nargin < 12
        early_stop_patience = 1000; % 默认早停耐心值
    end

    nodes = [start(1), start(2), 0, 0];  % 初始节点：[x, y, parent_idx, cost]

    % 起终点合法性检测
    if is_in_obstacle(start(1), start(2), rect_obstacles, circle_obstacles, inflate_radius)
        error('起点位于障碍物内！');
    end
    if is_in_obstacle(goal(1), goal(2), rect_obstacles, circle_obstacles, inflate_radius)
        error('终点位于障碍物内！');
    end

    best_goal_idx = -1;
    best_goal_cost = Inf;
    
    % 早停机制相关变量
    no_improvement_count = 0;  % 连续无改善的迭代次数
    last_best_cost = Inf;      % 上一次的最佳代价
    first_path_found = false;  % 是否已找到第一条路径
    
    fprintf('RRT*算法开始执行...\n');

    for iter = 1:max_iter
        % 采样
        if rand() < goal_bias
            x_rand = goal;
        else
            x_rand = [map_bounds(1) + rand() * (map_bounds(2) - map_bounds(1)), ...
                      map_bounds(3) + rand() * (map_bounds(4) - map_bounds(3))];
        end

        % 最近节点
        nearest_idx = find_nearest_node(nodes, x_rand);
        x_nearest = nodes(nearest_idx, 1:2);

        % 扩展
        x_new = extend_node(x_nearest, x_rand, step_size);

        % 碰撞检测
        if is_path_collision_free(x_nearest, x_new, rect_obstacles, circle_obstacles, inflate_radius)
            near_indices = find_near_nodes(nodes, x_new, search_radius);

            % 选父节点
            min_cost = nodes(nearest_idx, 4) + norm(x_nearest - x_new);
            best_parent_idx = nearest_idx;
            for i = 1:length(near_indices)
                near_idx = near_indices(i);
                x_near = nodes(near_idx, 1:2);
                if is_path_collision_free(x_near, x_new, rect_obstacles, circle_obstacles, inflate_radius)
                    cost = nodes(near_idx, 4) + norm(x_near - x_new);
                    if cost < min_cost
                        min_cost = cost;
                        best_parent_idx = near_idx;
                    end
                end
            end

            new_node = [x_new(1), x_new(2), best_parent_idx, min_cost];
            nodes = [nodes; new_node];
            new_node_idx = size(nodes, 1);

            % rewire
            for i = 1:length(near_indices)
                near_idx = near_indices(i);
                x_near = nodes(near_idx, 1:2);
                new_cost = min_cost + norm(x_new - x_near);
                if new_cost < nodes(near_idx, 4)
                    if is_path_collision_free(x_new, x_near, rect_obstacles, circle_obstacles, inflate_radius)
                        nodes(near_idx, 3) = new_node_idx;
                        nodes(near_idx, 4) = new_cost;
                        nodes = update_children_cost(nodes, near_idx);
                    end
                end
            end

            % 尝试连接目标
            if norm(x_new - goal) < step_size
                if is_path_collision_free(x_new, goal, rect_obstacles, circle_obstacles, inflate_radius)
                    goal_cost = min_cost + norm(x_new - goal);
                    if goal_cost < best_goal_cost
                        best_goal_cost = goal_cost;
                        goal_node = [goal(1), goal(2), new_node_idx, goal_cost];
                        if best_goal_idx > 0
                            % 替换之前的目标节点
                            nodes(best_goal_idx, :) = goal_node;
                        else
                            % 添加新的目标节点
                            nodes = [nodes; goal_node];
                            best_goal_idx = size(nodes, 1);
                        end
                        
                        % 第一次找到路径
                        if ~first_path_found
                            first_path_found = true;
                            fprintf('第%d次迭代找到第一条路径，代价: %.3f\n', iter, best_goal_cost);
                        end
                    end
                end
            end
            
            % 早停检查
            if first_path_found
                if best_goal_cost < last_best_cost
                    % 路径有改善，重置计数器
                    no_improvement_count = 0;
                    last_best_cost = best_goal_cost;
                    fprintf('第%d次迭代路径优化，新代价: %.3f\n', iter, best_goal_cost);
                else
                    % 路径无改善，增加计数器
                    no_improvement_count = no_improvement_count + 1;
                end
                
                % 检查是否达到早停条件
                if no_improvement_count >= early_stop_patience
                    fprintf('早停触发：连续%d次迭代无改善，在第%d次迭代停止\n', early_stop_patience, iter);
                    break;
                end
            end
        end
        
        % 每1000次迭代显示进度
        if mod(iter, 1000) == 0 && first_path_found
            fprintf('第%d次迭代，当前最佳代价: %.3f，无改善次数: %d/%d\n', ...
                iter, best_goal_cost, no_improvement_count, early_stop_patience);
        end
    end

    if best_goal_idx > 0
        [path_x, path_y] = reconstruct_path_rrt(nodes, best_goal_idx);
        if num_points > 0
            [path_x, path_y] = resample_path_rrt(path_x, path_y, num_points);
        end
        fprintf('算法完成，最终路径代价: %.3f\n', best_goal_cost);
    else
        path_x = [];
        path_y = [];
        warning('在最大迭代次数内未找到路径！');
    end
end

% --- 辅助函数 ---

function nearest_idx = find_nearest_node(nodes, x_rand)
    % 找到距离随机点最近的节点
    distances = sqrt(sum((nodes(:,1:2) - x_rand).^2, 2));
    [~, nearest_idx] = min(distances);
end

function x_new = extend_node(x_nearest, x_rand, step_size)
    % 从最近节点向随机点扩展固定步长
    direction = x_rand - x_nearest;
    distance = norm(direction);
    
    if distance <= step_size
        x_new = x_rand;
    else
        x_new = x_nearest + step_size * (direction / distance);
    end
end

function near_indices = find_near_nodes(nodes, x_new, search_radius)
    % 在搜索半径内找到所有邻近节点
    distances = sqrt(sum((nodes(:,1:2) - x_new).^2, 2));
    near_indices = find(distances <= search_radius);
end

function collision = is_in_obstacle(x, y, rect_obstacles, circle_obstacles, inflate_radius)
    % 检查点是否在障碍物内
    collision = false;
    
    % 检查矩形障碍物
    for k = 1:size(rect_obstacles, 1)
        rect = rect_obstacles(k, :);
        x_min = min(rect(1), rect(3));
        x_max = max(rect(1), rect(3));
        y_min = min(rect(2), rect(4));
        y_max = max(rect(2), rect(4));
        
        dist_to_rect = point_to_rectangle_distance(x, y, x_min, x_max, y_min, y_max);
        if dist_to_rect <= inflate_radius
            collision = true;
            return;
        end
    end
    
    % 检查圆形障碍物
    for k = 1:size(circle_obstacles, 1)
        circle = circle_obstacles(k, :);
        cx = circle(1);
        cy = circle(2);
        r = circle(3) + inflate_radius;
        
        if (x - cx)^2 + (y - cy)^2 <= r^2
            collision = true;
            return;
        end
    end
end

function collision_free = is_path_collision_free(x1, x2, rect_obstacles, circle_obstacles, inflate_radius)
    % 检查两点间路径是否无碰撞
    num_checks = ceil(norm(x2 - x1) / 0.1); % 每0.1单位检查一次
    
    for i = 0:num_checks
        t = i / num_checks;
        x_check = x1 + t * (x2 - x1);
        
        if is_in_obstacle(x_check(1), x_check(2), rect_obstacles, circle_obstacles, inflate_radius)
            collision_free = false;
            return;
        end
    end
    
    collision_free = true;
end

function nodes = update_children_cost(nodes, parent_idx)
    parent_cost = nodes(parent_idx, 4);
    parent_pos = nodes(parent_idx, 1:2);
    children_indices = find(nodes(:,3) == parent_idx);

    for i = 1:length(children_indices)
        child_idx = children_indices(i);
        child_pos = nodes(child_idx, 1:2);
        new_cost = parent_cost + norm(parent_pos - child_pos);
        nodes(child_idx, 4) = new_cost;
        nodes = update_children_cost(nodes, child_idx);  % 递归
    end
end

function [path_x, path_y] = reconstruct_path_rrt(nodes, goal_idx)
    % 重建从根节点到目标的路径
    path_x = [];
    path_y = [];
    
    current_idx = goal_idx;
    while current_idx ~= 0
        path_x = [nodes(current_idx, 1), path_x];
        path_y = [nodes(current_idx, 2), path_y];
        current_idx = nodes(current_idx, 3);
    end
end

function [resampled_x, resampled_y] = resample_path_rrt(x, y, n_points)
    % 路径重采样函数
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