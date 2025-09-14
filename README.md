## Geodesic-Based Path Planning for Port Transfer Robots on Riemannian Manifolds
> Published in *Expert Systems With Applications*, DOI: [10.1016/j.eswa.2025.129706](https://doi.org/10.1016/j.eswa.2025.129706)

### Core Features
- Constructs a complete and continuously differentiable potential field for the port's rectangular yard environment with circular obstacles.
- Formulates the robot's kinematic constraints and control costs as a metric tensor, combined with obstacle potential field coefficients to build a complete Riemannian metric.
- Introduces Geometric Heat Flow (GHF) as the solver for geodesic computation.

### Requirement
- Matlab (including PDEPE Solver)
- Python (matplotlib/shapely/numpy/scipy)

### Code Description
- `Astar.m`: Implements A* path planning to provide an initial solution for GHF.
- `compute_total_potential.m`: Generates a continuously differentiable obstacle potential field.
- `RRTS.m`: Implements RRT* with an early-stopping mechanism.
- `RRT_Planner`: Main code for the RRT* planner, including time statistics and visualization for multiple runs.
- `simulation.m`: Simulation code for open scenarios, including environment setup, Riemannian metric construction, and GHF solving.
- `simulation_add.m`: Additional simulation for open scenarios, with similar functionality as above.
- `visualization/visual.py`: Handles data visualization and partial performance metric calculations.

### Demonstration of Results
- Simulation Experiments
  ![ddd](/docs/6a.jpg)
  ![ddd](/docs/7a.jpg)
- Real-World Experiments - Planning
  ![ddd](/docs/10.jpg)
- Real-World Experiments - Feasibility Validation
  ![ddd](/docs/11.jpg)

### Discussion and Reflections
- There are several directions for optimization, such as obstacle representation, potential field construction, and solving methods. The current obstacle potential field gradient is still too steep, leading to convergence difficulties beyond a certain safety distance, which could be further optimized. Additionally, there are various methods for geodesic computation, such as cubic spline fitting, relaxation solvers based on error functions, or forward iteration with the Runge-Kutta method. Due to my limited mathematical expertise, these are not explored in depth here.
- Fundamentally, global path planning is a large-scale problem with strong terminal constraints. Forward iteration methods like Runge-Kutta cannot meet terminal constraints effectively. Discrete solvers like FMM or Heat Flow may lose precision and fail to strictly satisfy terminal constraints, requiring additional processing. In port path planning scenarios where real-time performance is not highly critical, GHF is the most straightforward approach that meets requirements without extra processing.
- Embedding various constraints into a Riemannian manifold aims to unify complex constraints into a single geometric plane for optimization. However, the solving details are much more complex than initially anticipated, and there is still much to learn. The current implementation is somewhat slow in terms of solving efficiency, and I hope to find further optimization strategies in the future.
- This code is an early version and does not include the robot's three-circle potential field superposition, though implementing this would be relatively straightforward.
> Initially, the implementation was done in Matlab, but due to issues at my organization, I couldn’t directly detail the Matlab solving process. As a result, I explored other methods to solve the GHF equation and ultimately chose Fipy as the solver library. Fipy is slightly faster than Matlab but has some peculiar bugs. In the end, I opted to open-source the early Matlab code. The core idea of Riemannian metric construction remains the same, and the GHF solver is a replaceable module that can be solved using other methods as well.

</details>


<details>
<summary>点击查看中文版</summary>

## 基于黎曼流形上测地线的港口转运机器人路径规划
> 发表于 *Expert Systems With Applications*，DOI：[10.1016/j.eswa.2025.129706](https://doi.org/10.1016/j.eswa.2025.129706)

### 基本功能
- 将港口的矩形堆场环境与圆形障碍物构建一个完整且连续可导的势场
- 将机器人运动学约束与控制代价构建为测度矩阵，结合障碍物势场系数构建完整黎曼测度
- 将 Geometric Heat Flow (GHF) 引入作为测地线的求解器

### 基本前置
- Matlab（包括 PDEPE Solver）
- Python（matplotlib/shapely/numpy/scipy）

### 代码说明
- `Astar.m`：用于求解 A* 路径规划结果，给 GHF 提供初始解
- `compute_total_potential.m`：用于生成连续可导障碍物势场
- `RRTS.m`：RRT* 的具体实现，包括早停机制
- `RRT_Planner`：RRT* 规划器应用主代码，包含多次运行的时间统计与可视化
- `simulation.m`：宽阔场景仿真代码，包含场景构建、黎曼测度构造与 GHF 求解
- `simulation_add.m`：宽敞场景新增场景仿真，具体内容同上
- `visualization/visual.py`：数据可视化与部分指标统计

### 效果演示
- 仿真实验
  ![ddd](/docs/6a.jpg)
  ![ddd](/docs/7a.jpg)
- 实际实验 - 规划实验
  ![ddd](/docs/10.jpg)
- 实际实验 - 可行性验证
  ![ddd](/docs/11.jpg)

### 讨论与思考
- 其实这个思路还有不少可以优化的方向，比如障碍物的表示，势场的构建形式，求解的方式等等。现有的障碍物势场梯度还是太大了，在安全距离以外一段距离就开始出现收敛困难的现象，这点还可以进一步优化。另外目前针对测地线求解的方法其实还有很多，比如三次样条曲线拟合/基于误差函数的松弛求解器/Runge–Kutta 法向前迭代等等，由于本人数学水平有限，不过多做探究
- 但是本质上全局路径规划问题是一个大尺度强终端约束的问题，因此 RK 法向前迭代法无法满足终端约束需求。另外，FMM，Heat Flow 等离散化的求解方式会损失精度，此外也不能严格满足终端需求，需要进行额外的处理。在港口路径这种实时性不要求特别高的场景下，GHF 是最不需要额外处理且一次就能满足需求的方式
- 把各个约束构建到黎曼流形里本质上是希望将复杂的各种约束统一到一个整体的几何平面中进行优化，但是具体求解细节比我一开始想的要复杂很多，还有很多需要学习的地方。目前我的实现在求解效率上还是有些慢，希望以后能够找到进一步优化的思路
- 这份代码由于是相对早期的版本，未加入机器人三圆势场叠加，但这一步实现起来相对容易
> 其实一开始实现就是使用 Matlab 实现的，结果发现由于所在单位的问题，没法直接写 Matlab 的求解细节，因此又重新找其他的方式来求解 GHF 方程，最后选定了 Fipy 作为求解库。求解速度其实比 Matlab 要快一点，但是会有些奇怪的 bug，最后还是选择开源了一开始的早期 Matlab 代码，本质上最核心的黎曼测度构建思路都是一样的，GHF 求解器其实是可以替换的模块，用别的方法也可以求解

</details>
