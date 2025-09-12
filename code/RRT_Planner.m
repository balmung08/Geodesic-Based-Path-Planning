% RRT*算法应用示例（含早停机制）
clear; clc; close all;

% 参数设置
start = [7, 22]; % 起点坐标
goal = [7, 12]; % 终点坐标

inflate_radius = 1.8; % 障碍物膨胀半径
max_iter = 30000; % 最大迭代次数
step_size = 1; % 步长
search_radius = 3; % 搜索半径
goal_bias = 0.1; % 目标偏置概率
map_bounds = [0, 50, 0, 26]; % 地图边界 [xmin, xmax, ymin, ymax]
num_points = 100; % 重采样后的路径点数

% 新增早停参数
early_stop_patience = 1000; % 连续多少次迭代无改善则停止

rect_obstacles = [
     0, 0, 3, 25;
     3, 0, 20, 3;
     28, 0, 43, 3;
     43, 0, 48, 25;
     13, 9, 35, 19;
     0, 25, 20, 28;
     3, 16, 13, 17;
     30, 19, 35, 23;
]; % [xmin, ymin, xmax, ymax]

circle_obstacles = [
     12, 10, 1;
     24, 8, 1;
     36, 20, 1;
];

% 运行RRT*算法（包含重采样和早停机制）
fprintf('=== RRT*算法参数设置 ===\n');
fprintf('起点: [%.1f, %.1f]\n', start(1), start(2));
fprintf('终点: [%.1f, %.1f]\n', goal(1), goal(2));
fprintf('最大迭代次数: %d\n', max_iter);
fprintf('早停耐心值: %d次迭代\n', early_stop_patience);
fprintf('步长: %.2f\n', step_size);
fprintf('搜索半径: %.2f\n', search_radius);
fprintf('目标偏置概率: %.2f\n', goal_bias);
fprintf('\n');

tic;
[path_x, path_y] = RRTS(...
    start, goal, rect_obstacles, circle_obstacles, ...
    inflate_radius, max_iter, step_size, search_radius, ...
    goal_bias, map_bounds, num_points, early_stop_patience);
computation_time = toc;

% 检查是否找到路径
if isempty(path_x)
    fprintf('RRT*算法未找到可行路径！\n');
    return;
end

% 绘制结果
figure('Position', [100, 100, 1000, 800]);
hold on;
axis equal;
grid on;

% === 障碍物绘制部分 ===
% 绘制原始矩形障碍物（灰色填充）
for i = 1:size(rect_obstacles, 1)
    rect = rect_obstacles(i, :);
    x_min = min(rect(1), rect(3));
    y_min = min(rect(2), rect(4));
    width = abs(rect(3) - rect(1));
    height = abs(rect(4) - rect(2));
    rectangle('Position', [x_min, y_min, width, height], ...
        'FaceColor', [0.7, 0.7, 0.7], 'EdgeColor', 'k', 'LineWidth', 1.5);
end

% 绘制原始圆形障碍物（灰色填充）
for i = 1:size(circle_obstacles, 1)
    circle = circle_obstacles(i, :);
    cx = circle(1);
    cy = circle(2);
    r = circle(3);
    rectangle('Position', [cx-r, cy-r, 2*r, 2*r], ...
        'Curvature', [1, 1], 'FaceColor', [0.7, 0.7, 0.7], 'EdgeColor', 'k', 'LineWidth', 1.5);
end

% 绘制膨胀后的矩形障碍物 (红色边框)
for i = 1:size(rect_obstacles, 1)
    rect = rect_obstacles(i, :);
    x_min = min(rect(1), rect(3)) - inflate_radius;
    y_min = min(rect(2), rect(4)) - inflate_radius;
    width = abs(rect(3)-rect(1)) + 2*inflate_radius;
    height = abs(rect(4)-rect(2)) + 2*inflate_radius;
    rectangle('Position', [x_min, y_min, width, height], ...
        'EdgeColor', [1, 0, 0], 'LineWidth', 1.5, 'LineStyle', '--');
end

% 绘制膨胀后的圆形障碍物 (红色边框)
for i = 1:size(circle_obstacles, 1)
    circle = circle_obstacles(i, :);
    cx = circle(1);
    cy = circle(2);
    r = circle(3) + inflate_radius;
    rectangle('Position', [cx-r, cy-r, 2*r, 2*r], ...
        'Curvature', [1, 1], 'EdgeColor', [1, 0, 0], 'LineWidth', 1.5, 'LineStyle', '--');
end

% 绘制路径
plot(path_x, path_y, 'b-o', 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', 'b');
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% 设置坐标轴
xlim([map_bounds(1), map_bounds(2)]);
ylim([map_bounds(3), map_bounds(4)]);

% 添加障碍物说明
h = zeros(4, 1);
h(1) = plot(NaN, NaN, 's', 'MarkerSize', 10, 'MarkerFaceColor', [0.7, 0.7, 0.7], 'MarkerEdgeColor', 'k');
h(2) = plot(NaN, NaN, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.7, 0.7, 0.7], 'MarkerEdgeColor', 'k');
h(3) = plot(NaN, NaN, '--r', 'LineWidth', 1.5);
h(4) = plot(NaN, NaN, 'b-o', 'LineWidth', 2);

% 添加图例和标题
legend(h, {'矩形障碍物', '圆形障碍物', '膨胀边界', 'RRT*路径'}, 'Location', 'bestoutside');
title(sprintf('RRT*路径规划 (早停: %d次无改善)', early_stop_patience));
xlabel('X'); ylabel('Y');

hold off;

% 显示算法性能信息
fprintf('\n=== RRT*算法性能统计 ===\n');
fprintf('计算时间: %.3f 秒\n', computation_time);
fprintf('原始路径点数: %d\n', length(path_x));
fprintf('重采样后点数: %d\n', num_points);
fprintf('早停耐心值: %d次迭代\n', early_stop_patience);

% 计算路径长度
if ~isempty(path_x)
    path_length = 0;
    for i = 1:length(path_x)-1
        path_length = path_length + sqrt((path_x(i+1)-path_x(i))^2 + (path_y(i+1)-path_y(i))^2);
    end
    fprintf('路径总长度: %.3f\n', path_length);
end

% 多次运行性能比较（测试早停机制效果）
fprintf('\n=== 不同早停设置的性能比较 ===\n');
early_stop_values = [1000,2000]; % 不同的早停耐心值
num_runs = 5; % 每个设置运行次数

results = struct();
for e = 1:length(early_stop_values)
    patience = early_stop_values(e);
    fprintf('\n--- 早停耐心值: %d ---\n', patience);
    
    times = [];
    lengths = [];
    success_count = 0;
    
    for run = 1:num_runs
        fprintf('运行 %d/%d... ', run, num_runs);
        tic;
        [temp_path_x, temp_path_y] = RRTS(...
            start, goal, rect_obstacles, circle_obstacles, ...
            inflate_radius, max_iter, step_size, search_radius, ...
            goal_bias, map_bounds, 0, patience); % num_points=0 for faster computation
        run_time = toc;
        
        if ~isempty(temp_path_x)
            success_count = success_count + 1;
            temp_length = 0;
            for i = 1:length(temp_path_x)-1
                temp_length = temp_length + sqrt((temp_path_x(i+1)-temp_path_x(i))^2 + (temp_path_y(i+1)-temp_path_y(i))^2);
            end
            times = [times, run_time];
            lengths = [lengths, temp_length];
            fprintf('成功 (时间: %.2fs, 长度: %.2f)\n', run_time, temp_length);
        else
            fprintf('失败\n');
        end
    end
    
    results(e).patience = patience;
    results(e).success_rate = success_count / num_runs * 100;
    results(e).avg_time = mean(times);
    results(e).avg_length = mean(lengths);
    results(e).std_time = std(times);
    results(e).std_length = std(lengths);
    
    fprintf('总结: 成功率 %.1f%%, 平均时间 %.2f±%.2fs, 平均长度 %.2f±%.2f\n', ...
        results(e).success_rate, results(e).avg_time, results(e).std_time, ...
        results(e).avg_length, results(e).std_length);
end

% 绘制性能对比图
figure('Position', [200, 200, 1200, 400]);

subplot(1, 3, 1);
bar([results.success_rate]);
set(gca, 'XTickLabel', {results.patience});
xlabel('早停耐心值');
ylabel('成功率 (%)');
title('成功率对比');
grid on;

subplot(1, 3, 2);
errorbar([results.avg_time], [results.std_time], 'o-', 'LineWidth', 2);
set(gca, 'XTickLabel', {results.patience});
xlabel('早停耐心值');
ylabel('计算时间 (秒)');
title('计算时间对比');
grid on;

subplot(1, 3, 3);
errorbar([results.avg_length], [results.std_length], 'o-', 'LineWidth', 2);
set(gca, 'XTickLabel', {results.patience});
xlabel('早停耐心值');
ylabel('路径长度');
title('路径长度对比');
grid on;

% 早停机制使用建议
fprintf('\n=== 早停机制使用建议 ===\n');
fprintf('1. 早停耐心值设置建议：\n');
fprintf('   - 追求速度: 500-1000次迭代\n');
fprintf('   - 平衡速度和质量: 1000-2000次迭代\n');
fprintf('   - 追求路径质量: 2000-5000次迭代\n');
fprintf('2. 早停耐心值越小，计算速度越快，但路径可能不够优化\n');
fprintf('3. 对于复杂环境，建议适当增加早停耐心值\n');
fprintf('4. 可以根据实际应用需求调整early_stop_patience参数\n');

% 保存结果
save('RRT_1.mat', 'path_x', 'path_y', 'results', 'early_stop_patience', 'computation_time');