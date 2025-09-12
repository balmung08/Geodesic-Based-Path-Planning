import matplotlib.pyplot as plt
from shapely.geometry import box, Point
from shapely.ops import unary_union
import numpy as np
import scipy.io
import matplotlib.patches as patches


# 设置学术论文风格
plt.rcParams.update({
    'font.size': 11,
    'font.family': 'Calibri',
    'axes.linewidth': 1.0,
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10,
    'figure.titlesize': 14,
    'lines.linewidth': 2.0,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'grid.linewidth': 0.5
})


def compute_discrete_path_metrics(x, y, yaw=None):
    """计算路径的总长度、最大曲率和平均曲率，支持倒退路径"""
    x = np.asarray(x)
    y = np.asarray(y)

    # 计算每段的切向角 theta
    theta = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))

    # 计算每段的长度（用于 dtheta/ds）
    ds = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    if yaw is None:
        dtheta = np.diff(theta)
    else:
        dyaw = np.diff(yaw)
        dtheta = np.arctan2(np.sin(dyaw), np.cos(dyaw))  # 修正角度跳变
    # 避免除以零
    ds[ds < 1e-8] = 1e-8

    # 曲率定义为 dtheta / ds
    curvature = np.zeros_like(x)
    curvature[1:] = dtheta / ds
    curvature[0] = curvature[1]  # 首点补值

    # 路径总长度
    total_length = np.sum(ds)

    # 最大和平均曲率（绝对值）
    max_curvature = np.max(np.abs(curvature))
    mean_curvature = np.mean(np.abs(curvature))
    top3 = sorted(np.abs(curvature), reverse=True)[:3]
    return total_length, max_curvature, mean_curvature



def compute_obstacle_distances(x, y, rects, circles):
    """计算到障碍物的距离"""
    rect_shapes = [box(*rect) for rect in rects]
    circle_shapes = [Point(cx, cy).buffer(r) for cx, cy, r in circles]

    all_obstacles = rect_shapes + circle_shapes
    merged_obstacles = unary_union(all_obstacles)

    distances = []
    for xi, yi in zip(x, y):
        pt = Point(xi, yi)
        dist = pt.distance(merged_obstacles)
        distances.append(dist)

    return np.array(distances), np.mean(distances)


# 学术论文配色方案（符合期刊要求）
ACADEMIC_COLORS = {
    'obstacle_original': '#4A4A4A',  # 中性深灰
    'obstacle_inflated': '#D9D9D9',  # 浅灰
    'obstacle_edge': '#6B6B6B',  # 中灰
    'path_proposed': '#E64DB3',  # 橙红色（提议方法）
    'path_A': '#FF3E00',  # 道奇蓝（基准方法）
    'path_RRT': '#548235',  # 道奇蓝（基准方法）
    'path_HA': '#2E75B6',  # 道奇蓝（基准方法）
    'start_goal': '#228B22',  # 森林绿
    'background': '#FFFFFF',  # 白色背景
    'grid': '#D3D3D3',  # 浅灰
    'text': '#000000'  # 黑色文本
}

# =========================
# 障碍物定义
# =========================

rects = np.array([
    [0, 0, 3, 25],
    [3, 0, 20, 3],
    [28, 0, 43, 3],
    [43, 0, 48, 25],
    [13, 9, 35, 19],
    [0, 25, 20, 28],
    [3, 16, 13, 17],
    [30, 19, 35, 23],
])
circles = np.array([
    [12,10,1],
    [24,8,1],
    [36,20,1],
])
inflation_radius = 1.8

# 构造膨胀区域
inflated_shapes = []
for rect in rects:
    xmin, ymin, xmax, ymax = rect
    rect_shape = box(xmin, ymin, xmax, ymax)
    inflated_shapes.append(rect_shape.buffer(inflation_radius, cap_style=1, join_style=1))

for cx, cy, r in circles:
    circle_shape = Point(cx, cy).buffer(r + inflation_radius)
    inflated_shapes.append(circle_shape)

merged_obstacles = unary_union(inflated_shapes)

mat = scipy.io.loadmat('../code/multi_path_data.mat')
path1_x,path1_y = (mat['path_x_smooth'].flatten(), mat['path_y_smooth'].flatten())
path2_x,path2_y = (mat['X'].flatten(), mat['Y'].flatten())
mat = scipy.io.loadmat('../code/RRT_1.mat')
path3_x,path3_y = (mat['path_x'].flatten(), mat['path_y'].flatten())


# 计算路径指标
length1, max_k1, mean_k1 = compute_discrete_path_metrics(path1_x, path1_y)
distances1, avg_dist1 = compute_obstacle_distances(path1_x, path1_y, rects, circles)

length2, max_k2, mean_k2 = compute_discrete_path_metrics(path2_x, path2_y)
distances2, avg_dist2 = compute_obstacle_distances(path2_x, path2_y, rects, circles)

length3, max_k3, mean_k3 = compute_discrete_path_metrics(path3_x, path3_y)
distances3, avg_dist3 = compute_obstacle_distances(path3_x, path3_y, rects, circles)



print("{:<12} {:>10} {:>12} {:>14} {:>16} {:>16}".format(
    "Method", "Length", "Max Curv", "Mean Curv", "Min Obs Dist", "Avg Obs Dist"))
print("=" * 100)
print("{:<12} {:>10.4f} {:>12.4f} {:>14.4f} {:>16.4f} {:>16.2f}".format(
    "A*", length1, max_k1, mean_k1, np.min(distances1), avg_dist1))
print("{:<12} {:>10.4f} {:>12.4f} {:>14.4f} {:>16.4f} {:>16.2f}".format(
    "Geodesic", length2, max_k2, mean_k2, np.min(distances2), avg_dist2))
print("{:<12} {:>10.4f} {:>12.4f} {:>14.4f} {:>16.4f} {:>16.2f}".format(
    "RRT*", length3, max_k3, mean_k3, np.min(distances3), avg_dist3))


# =========================
# 可视化
# =========================
fig, ax = plt.subplots(figsize=(10, 6), dpi=300)
ax.set_aspect('equal')

# 绘制膨胀区域（安全区域）
if merged_obstacles.geom_type == 'Polygon':
    x, y = merged_obstacles.exterior.xy
    ax.fill(x, y, color=ACADEMIC_COLORS['obstacle_inflated'], alpha=0.6,
            edgecolor=ACADEMIC_COLORS['obstacle_edge'], linewidth=1.5,
            linestyle='--', label='Safety margin')
elif merged_obstacles.geom_type == 'MultiPolygon':
    for i, geom in enumerate(merged_obstacles.geoms):
        x, y = geom.exterior.xy
        label = 'Safety margin' if i == 0 else None
        ax.fill(x, y, color=ACADEMIC_COLORS['obstacle_inflated'], alpha=0.4,
                edgecolor=ACADEMIC_COLORS['obstacle_edge'], linewidth=1.5,
                linestyle='--', label=label)

# 绘制原始障碍物
for i, rect in enumerate(rects):
    xmin, ymin, xmax, ymax = rect
    rect_patch = patches.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                                   facecolor=ACADEMIC_COLORS['obstacle_original'],
                                   edgecolor='black', linewidth=1.5, alpha=0.8,
                                   label='Obstacles' if i == 0 else None)
    ax.add_patch(rect_patch)

for i, (cx, cy, r) in enumerate(circles):
    circle = Point(cx, cy).buffer(r)
    x, y = circle.exterior.xy
    ax.fill(x, y, color=ACADEMIC_COLORS['obstacle_original'], alpha=0.8,
            edgecolor='black', linewidth=1.5)

# 绘制路径
ax.plot(path1_x, path1_y, color=ACADEMIC_COLORS['path_A'],
        linewidth=2, linestyle='--', label='Baseline A', zorder=5)
ax.plot(path2_x, path2_y, color=ACADEMIC_COLORS['path_proposed'],
        linewidth=3, linestyle='-', label='Proposed', zorder=5)
ax.plot(path3_x, path3_y, color=ACADEMIC_COLORS['path_RRT'],
        linewidth=2, linestyle='--', label='Baseline RRT', zorder=5)

# 绘制起点和终点
def draw_academic_marker(ax, pose, color, marker_type, label):
    x, y, theta, _ = pose
    if marker_type == 'start':
        ax.scatter(x, y, s=60, c=color, marker='o', edgecolor='black',
                   linewidth=2, zorder=10, label=label)
    else:  # goal
        ax.scatter(x, y, s=60, c=color, marker='s', edgecolor='black',
                   linewidth=2, zorder=10, label=label)

    # 方向箭头
    dx = np.cos(theta) * 1.5
    dy = np.sin(theta) * 1.5
    ax.arrow(x, y, dx, dy, head_width=0.6, head_length=0.8,
             fc=color, ec='black', linewidth=1, zorder=11)

draw_academic_marker(ax, [7, 22, 0, 0], ACADEMIC_COLORS['start_goal'], 'start', 'Start')
draw_academic_marker(ax, [7, 12, np.pi / 2, 0], ACADEMIC_COLORS['start_goal'], 'goal', 'Goal')

# 网格设置
ax.grid(True, color=ACADEMIC_COLORS['grid'], alpha=0.5, linewidth=0.5)
ax.set_axisbelow(True)

ax.set_xlim(-4, 52)
ax.set_ylim(-3, 28)

ax.set_xlabel('X (m)', fontsize=16,fontweight='bold')
ax.set_ylabel('Y (m)', fontsize=16,fontweight='bold')


ax.tick_params(axis='both', which='major', labelsize=14, width=1.5)
for label in ax.get_xticklabels() + ax.get_yticklabels():
    label.set_fontweight('bold')

# 图例设置
legend = ax.legend(loc='upper right', frameon=True, fancybox=False, shadow=False,
                   framealpha=0.2, facecolor='white', edgecolor='black', fontsize=8)
legend.get_frame().set_linewidth(1.0)


plt.show()