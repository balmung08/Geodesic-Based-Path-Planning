function [U_func, gradU_func] = compute_total_potential(rects, circles, params)
% 返回一个匿名函数 U_func(p) 用于查询任意点的势场值
% p: [2×1] 位置向量 [x; y]
    U_func = @(p) compute_potential(p, rects, circles, params);
    % 梯度用有限差分计算，步长可调节
    h = 1e-5;
    gradU_func = @(p) [
        (compute_potential([p(1)+h; p(2)], rects, circles, params) - compute_potential([p(1)-h; p(2)], rects, circles, params)) / (2*h);
        (compute_potential([p(1); p(2)+h], rects, circles, params) - compute_potential([p(1); p(2)-h], rects, circles, params)) / (2*h);
    ];
end

% function U_func = compute_total_potential(rects, circles, params)
% % 返回一个匿名函数 U_func(p) 用于查询任意点的势场值
% % p: [2×1] 位置向量 [x; y]
%     U_func = @(p) compute_potential(p, rects, circles, params);
% end

function U = compute_potential(p, rects, circles, params)
x = p(1); y = p(2);
U = 0;
R = params.R;

% % 矩形障碍
% for i = 1:size(rects,1)
%     xmin = rects(i,1); xmax = rects(i,2);
%     ymin = rects(i,3); ymax = rects(i,4);
% 
%     edges = {
%         [xmin, xmax; ymin, ymin];
%         [xmin, xmax; ymax, ymax];
%         [xmin, xmin; ymin, ymax];
%         [xmax, xmax; ymin, ymax];
%     };
% 
%     for j = 1:4
%         pt1 = edges{j}(:,1); pt2 = edges{j}(:,2);
%         [d, valid] = point_to_edge_distance(p, pt1, pt2);
%         d_eff = d - R;
%         if valid
%             if d_eff <= 0
%                 U = 1000;
%             elseif d_eff < R
%                 U = U + min(100 * ((d_eff - R)/R)^2, 100);
%             end
%         end
%     end
% 
%     corners = [xmin, ymin; xmin, ymax; xmax, ymin; xmax, ymax];
%     for k = 1:4
%         q = corners(k,:)';
%         d_eff = norm(p - q) - R;
%         if d_eff < 0
%             U = U + 1000;
%         elseif d_eff < R
%             U = U + min(100 * ((d_eff - R)/R)^2, 100);
%         end
%     end
%     U = min(100,U);
% end

% 改进的矩形障碍处理
for i = 1:size(rects,1)
    xmin = rects(i,1); xmax = rects(i,3);
    ymin = rects(i,2); ymax = rects(i,4);
    
    % 点到矩形的最近距离
    dx = max([xmin - x, 0, x - xmax]);
    dy = max([ymin - y, 0, y - ymax]);
    d = sqrt(dx^2 + dy^2);
    
    d_eff = d - R;
    if d_eff < 0
        U = U + 10;  % 可调惩罚
    elseif d_eff < R
        U = U + 10 * ((d_eff - R)/R)^2;
    end
end

% 圆形障碍
for i = 1:size(circles, 1)
    c = circles(i, 1:2)';
    r_obs = circles(i, 3);
    d_eff = norm(p - c) - r_obs - R;
    if d_eff < 0
        U = U + 10;
    elseif d_eff < R
        U = U + min(10 * ((d_eff - R)/R)^2, 10);
    end
end

end

function [d, valid] = point_to_edge_distance(p, a, b)
ap = p - a;
ab = b - a;
t = dot(ap, ab) / dot(ab, ab);
if t >= 0 && t <= 1
    proj = a + t * ab;
    d = norm(p - proj);
    valid = true;
else
    d = 0;
    valid = false;
end
end






