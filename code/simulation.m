function simulation()
% solve: 求解基于Christoffel符号的测地线 PDE 问题并绘图，含障碍物展示

    % ========================
    % 统一障碍物数据定义
    % ========================
    obstacle_data = struct();
    obstacle_data.rects = [
     0, 0, 5, 25;
     5, 0, 20, 8;
     28, 0, 43, 8;
     43, 0, 48, 25;
     13, 16, 35, 25;
    ];  % [xmin, ymin, xmax, ymax]
    obstacle_data.circles = [
     13, 10, 1;
     24, 14, 1;
    ];
    obstacle_data.params.R = 1.5;        % 势场半径参数
    obstacle_data.inflation_radius = 1.8; % 障碍物膨胀半径（绘图用）
    obstacle_data.show_original = true; % 显示原始障碍物轮廓

    boundary_conditions = struct();
    boundary_conditions.left = [9; 25; -pi/2; 0];    % 左边界值
    boundary_conditions.right = [24; 0; -pi/2; 0]; % 右边界值
    % boundary_conditions.right = [39; 25; pi/2; 0]; % 右边界值
    boundary_conditions.left_q = [0; 0; 0; 0];   % 左边界导数系数
    boundary_conditions.right_q = [0; 0; 0; 0];  % 右边界导数系数
    
    % Astar求初始路径
    start = [boundary_conditions.left(1), boundary_conditions.left(2)];          % 起点坐标
    goal = [boundary_conditions.right(1), boundary_conditions.right(2)];         % 终点坐标
    inflate_radius = 1.8;    % 障碍物膨胀半径
    grid_resolution = 0.05;   % 网格分辨率
    map_bounds = [-50, 50, -50, 25]; % 地图边界 [xmin, xmax, ymin, ymax]
    num_points = 100;         % 重采样后的路径点数,与PDE离散的x个数统一
    tic;
    [path_x, path_y] = AStar(...
    start, goal, obstacle_data.rects, obstacle_data.circles, ...
    inflate_radius, grid_resolution, map_bounds, num_points);
    toc;
    % 构造 u0_list 用于 PDE 初始值
    global u0_list N
    N = length(path_x);
    s = linspace(0, 1, N);
    spx = spap2(round(N/4), 4, s, path_x);
    spy = spap2(round(N/4), 4, s, path_y);
    
    s_dense = linspace(0, 1, N);
    path_x_smooth = fnval(spx, s_dense);
    path_y_smooth = fnval(spy, s_dense);
    
    dx = fnval(fnder(spx), s_dense);
    dy = fnval(fnder(spy), s_dense);
    theta_smooth = atan2(dy, dx);
    dtheta_ds = gradient(theta_smooth, s_dense);  % 数值一阶导

    L = 5.44;  % 车辆轴距（可根据实际设定）
    delta_smooth = atan(L * dtheta_ds);  % 车辆转角估算
    % u0_list = [path_x_smooth; path_y_smooth; theta_smooth; delta_smooth];
   
    u0_list = [path_x_smooth; path_y_smooth; theta_smooth; zeros(1, N)];
    % boundary_conditions.left(3) = theta_smooth(1);
    % boundary_conditions.right(3) = theta_smooth(end);

    % 构建x和t的区间
    % clc;
    tmax = 1;
    m = 0;
    x = linspace(0, 1, num_points);
    t = linspace(0, tmax, 100);



    options = odeset('RelTol', 1e-1, 'AbsTol', 1e-3, 'MaxStep', 0.05);
    tic;
    % 求解 PDE 方程
    sol = pdepe(m, @(x,t,u,DuDx) mypdexpde(x,t,u,DuDx,obstacle_data), @mypdexic, ...
        @(xl,ul,xr,ur,t) mypdexbc(xl,ul,xr,ur,t,boundary_conditions), x, t,options);

    toc;

    % 提取状态量
    u1 = sol(:,:,1);
    u2 = sol(:,:,2);

    % 绘图 - 轨迹及障碍物
    figure;
    subplot(2,1,1);
    X = u1(1,:); Y = u2(1,:);
    h1 = plot(X, Y, 'b', 'LineWidth', 2); hold on;
    h2 = plot(path_x_smooth, path_y_smooth, 'r', 'LineWidth', 2); hold on;
    title('Evolvement of boxcar trajectory in xy-plane');
    axis([-3, 3, -3, 3]); axis equal; grid on;

    % 绘制障碍物
    draw_obstacles(obstacle_data);

    % 动画更新
    for i = 1:size(u1,1)
        X = u1(i,:); h1.XDataSource = 'X';
        Y = u2(i,:); h1.YDataSource = 'Y';
        refreshdata(h1, 'caller');
        drawnow;
    end
    theta_last = sol(end,:,3);
    delta_last = sol(end,:,4);
    total_steering = sum(abs(delta_last));
    disp(total_steering);
    % theta 与 delta 曲线
    subplot(2,1,2);
    plot(x, sol(end,:,3)); hold on;
    plot(x, sol(end,:,4));
    % plot(x, dtheta_ds);
    xlabel('s'); ylabel('rad');
    legend('\theta', '\delta');
    title('Data');
    save('multi_path_data.mat', ...
    'path_x_smooth', 'path_y_smooth', ...
    'X', 'Y');
end

% ========================
% 障碍物绘制函数
% ========================
function draw_obstacles(obstacle_data)
    inflation_radius = obstacle_data.inflation_radius;
    
    % 绘制矩形障碍物（膨胀版本）
    rects = obstacle_data.rects;
    for i = 1:size(rects,1)
        xmin = rects(i,1) - inflation_radius; 
        xmax = rects(i,3) + inflation_radius;
        ymin = rects(i,2) - inflation_radius; 
        ymax = rects(i,4) + inflation_radius;
        fill([xmin xmax xmax xmin], [ymin ymin ymax ymax], ...
            [0.1 0.1 0.8], 'FaceAlpha', 0.8, 'EdgeColor', 'none');
    end

    % 绘制圆形障碍物（膨胀版本）
    circles = obstacle_data.circles;
    theta = linspace(0, 2*pi, 100);
    for i = 1:size(circles,1)
        cx = circles(i,1); 
        cy = circles(i,2); 
        r = circles(i,3) + inflation_radius; % 半径增加膨胀值
        x_c = cx + r * cos(theta);
        y_c = cy + r * sin(theta);
        fill(x_c, y_c, [0.1 0.1 0.8], 'FaceAlpha', 0.8, 'EdgeColor', 'none');
    end
    
    % 可选：绘制原始障碍物轮廓（用于对比）
    if isfield(obstacle_data, 'show_original') && obstacle_data.show_original
        % 原始矩形轮廓
        for i = 1:size(rects,1)
            xmin = rects(i,1); xmax = rects(i,3);
            ymin = rects(i,2); ymax = rects(i,4);
            plot([xmin xmax xmax xmin xmin], [ymin ymin ymax ymax ymin], ...
                'r--', 'LineWidth', 1.5);
        end
        
        % 原始圆形轮廓
        for i = 1:size(circles,1)
            cx = circles(i,1); cy = circles(i,2); r = circles(i,3);
            x_c = cx + r * cos(theta);
            y_c = cy + r * sin(theta);
            plot(x_c, y_c, 'r--', 'LineWidth', 1.5);
        end
    end
end



% ========================
% 子函数：PDE 方程定义
% ========================
function [c,f,s] = mypdexpde(x,t,u,DuDx,obstacle_data)
    if any(isnan(u)) || any(isnan(DuDx)) || any(isinf(u)) || any(isinf(DuDx))
        error("爆炸: u 或 DuDx 中存在 NaN/Inf，t=%.5f, x=%.5f", t, x);
    end
    lambda1 = 20;
    lambda2 = 20;
    lambda3 = 1;
    L = 5.44;
    delta_sub = pi/6;
    delta_max = pi/4;

    % 控制角代价项
    if u(4) > delta_sub
        ratio = (u(4) - delta_sub) / (u(4) - delta_max);
        c = 1 + lambda3 * ratio^2;
        dc = 2 * lambda3 * (u(4) - delta_sub) * (delta_sub - delta_max) / (u(4) - delta_max)^3;
    elseif u(4) < -delta_sub
        ratio = (u(4) + delta_sub) / (u(4) + delta_max);
        c = 1 + lambda3 * ratio^2;
        dc = 2 * lambda3 * (u(4) + delta_sub) * (-delta_sub - delta_max) / (u(4) + delta_max)^3;
    else
        c = 1;
        dc = 0;
    end

    % 度量矩阵 G
    G = [
        sin(u(3))^2 * (lambda1 + 1) + (cos(u(3))^2 * (4*lambda2 * tan(u(4))^2 + L^2)) / L^2, ...
        (cos(u(3)) * sin(u(3)) * (4*lambda2 * tan(u(4))^2 - lambda1 * L^2)) / L^2, ...
        -2 * lambda2 * cos(u(3)) * tan(u(4)) / L, ...
        0;
    
        (cos(u(3)) * sin(u(3)) * (4*lambda2 * tan(u(4))^2 - lambda1 * L^2)) / L^2, ...
        cos(u(3))^2 * (lambda1 + 1) + (sin(u(3))^2 * (4*lambda2 * tan(u(4))^2 + L^2)) / L^2, ...
        -2 * lambda2 * sin(u(3)) * tan(u(4)) / L, ...
        0;
    
        -2 * lambda2 * cos(u(3)) * tan(u(4)) / L, ...
        -2 * lambda2 * sin(u(3)) * tan(u(4)) / L, ...
        lambda2, ...
        0;
    
        0, 0, 0, c;
    ];      
    
    pG=zeros(4);
    
    pG(:,:,3) = [
        -(2*cos(u(3))*sin(u(3))*(4*lambda2*tan(u(4))^2 - L^2*lambda1))/L^2,       (cos(2*u(3))*(4*lambda2*tan(u(4))^2 - L^2*lambda1))/L^2,    (2*lambda2*sin(u(3))*tan(u(4)))/L, 0;
        (cos(2*u(3))*(4*lambda2*tan(u(4))^2 - L^2*lambda1))/L^2,               (2*cos(u(3))*sin(u(3))*(4*lambda2*tan(u(4))^2 - L^2*lambda1))/L^2, -(2*lambda2*cos(u(3))*tan(u(4)))/L, 0;
        (2*lambda2*sin(u(3))*tan(u(4)))/L,                                     -(2*lambda2*cos(u(3))*tan(u(4)))/L,                         0, 0;
        0,                                                                 0,                                                       0, 0;
    ];
    
    pG(:,:,4) = [
        (8*lambda2*cos(u(3))^2*tan(u(4))*(tan(u(4))^2 + 1))/L^2,        (8*lambda2*cos(u(3))*sin(u(3))*sin(u(4)))/(L^2*cos(u(4))^3),     -(2*lambda2*cos(u(3)))/(L*cos(u(4))^2), 0;
        (8*lambda2*cos(u(3))*sin(u(3))*sin(u(4)))/(L^2*cos(u(4))^3),      (8*lambda2*sin(u(3))^2*tan(u(4))*(tan(u(4))^2 + 1))/L^2,       (2*lambda2*sin(u(3)))/(L*(sin(u(4))^2 - 1)), 0;
        -(2*lambda2*cos(u(3)))/(L*cos(u(4))^2),                       (2*lambda2*sin(u(3)))/(L*(sin(u(4))^2 - 1)),                0, 0;
        0,                                                        0,                                                     0,     dc;
    ];


    % 势场系数当作函数
    [U_func, gradU_func] = compute_total_potential(obstacle_data.rects, obstacle_data.circles, obstacle_data.params);
    V = max(1,U_func([u(1); u(2)]));

    grad = gradU_func([u(1); u(2)]);
    gradX = grad(1);
    gradY = grad(2);
    % max_grad = 100000;
    % gradX = max(-max_grad, min(max_grad, gradX));
    % gradY = max(-max_grad, min(max_grad, gradY));
    pG(:,:,1)=gradX*G;
    pG(:,:,2)=gradY*G;
    pG(:,:,3)=V*pG(:,:,3);
    pG(:,:,4)=V*pG(:,:,4);

    % Christoffel 符号计算
    invG = pinv(G)/V;
    Chris = zeros(size(pG));
    for i = 1:4
        for j = 1:4
            for k = 1:4
                for l = 1:4
                    Chris(i,j,k) = Chris(i,j,k) + 0.5 * invG(i,l) * ...
                        (pG(l,j,k) + pG(l,k,j) - pG(j,k,l));
                end
            end
        end
    end

    % PDE 标准项
    c = ones(4,1);
    f = DuDx;
    s = zeros(4,1);
    for i = 1:4
        s(i) = DuDx' * squeeze(Chris(i,:,:)) * DuDx;
    end
end


% ========================
% 边界条件
% ========================
function [pl, ql, pr, qr] = mypdexbc(xl, ul, xr, ur, t, boundary_conditions)
    % 使用在 solve 函数中定义的边界条件
    pl = ul - boundary_conditions.left;
    ql = boundary_conditions.left_q;
    pr = ur - boundary_conditions.right;
    qr = boundary_conditions.right_q;
end

% ========================
% 初始条件
% ========================
function u0 = mypdexic(x)
    global u0_list N

    % 将x ∈ [0,1] 映射为索引
    i = round(x * (N - 1)) + 1;

    % 边界处理，确保索引合法
    i = max(1, min(N, i));

    u0 = u0_list(:, i);  % 提取第i个状态向量
end