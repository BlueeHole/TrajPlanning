# TrajPlanning

## 介绍

- 为RM写的路径规划
- 功能是给定起始、结束点，规划出可以无障碍平滑高效到达的每一时刻的速度集合


## 使用说明

- 运行main.py
- 输入
    - 加载一张`map.jpg`
    - 在窗口中点一个点，作为目标点（起始点默认为左上角）
- 输出
    - 程序会使用A*规划一条path
    - 在此基础上，对x，y方向**分别单独**使用闭式求解Minimum-Snap的算法生成一条二维trajectory
    - 可视化中画出的是x，y的速度
    - 也提供了在图中画出轨迹的接口，为`multi_seg_traj_generator.py`中的visualize_traj函数


## 软件架构

- 软件架构说明
    - `map_manager.py`
        - 管理地图，说实话有点鸡肋，而且我还没太搞明白多线程图像刷新是咋个刷新法（OpenCV & 多线程）
    - `multi_seg_traj_generator.py`
        - traj_generator(path_list)
            - 本模块对外的接口，给定一个path，会生成通过path（**每10个点取1个点，这个待优化**）中某些点、连接处各阶导数连续、整体Snap平方最小的一条由多项式组成的轨迹，重点是为了得到速度给下层
        - MinimumSnapCloseformSolver(waypoints, ts, n_seg, n_order)
            - 闭式求解一维Minimum-Snap问题的具体函数，提供路径点、时间划分、轨迹段数、轨迹多项式阶数
            - 调用`getQ`, `getM`, `getCt`函数，计算出R矩阵，再分块、求导，得出最终dF
    - `main.py`
        - While循环中不断监听鼠标动作
            - 一旦左键单击，进入setGoal函数
            - 进行路径搜索，随后进行轨迹规划以及可视化

## TODOs

- 避障
- QP法求解
- 哈深的方法，更简单的规划方法
- 考虑动力学约束的方法
- 写文档，整理遇到的问题，主要有几个点
    - OpenCV与多线程
    - matlab转python过程中的问题，主要是numpy矩阵运算，矩阵和数组的区别
    - 调试方法（类似对拍，用matlab的正确程序生成结果，用python生成，然后写个py脚本对比结果来验证正确性）
    - 心态（不要畏难）