import numpy as np
from numpy.linalg import  solve
import matplotlib.pyplot as plt
from matplotlib import gridspec


def s(c, t):
    return c[0]*t**5 + c[1]*t**4 + c[2]*t**3 + c[3]*t**2 + c[4]*t + c[5]
def d_s(c, t):
    return 5*c[0]*t**4 + 4*c[1]*t**3 + 3*c[2]*t**2 + 2*c[3]*t + c[4]
def dd_s(c, t):
    return 20*c[0]*t**3 + 12*c[1]*t**2 + 6*c[2]*t + 2*c[3]

# 参考连接：https://blog.csdn.net/robinvista/article/details/89775112
def traj_plan(waypoints, boundary_cond, boundary_cond_y, T):
    A = np.mat([[0,      0,    0,    0, 0, 1],
               [T**5, T**4, T**3, T**2, T, 1],
               [0,       0,    0,    0, 1, 0],
               [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
               [0,       0,    0,    1, 0, 0],
               [20*T**3, 12*T**2, 6*T, 2, 0, 0]
                ])
    B = np.mat(boundary_cond).T
    c = solve(A, B)
    B_y = np.mat(boundary_cond_y).T
    c_y = solve(A, B_y)
    print(c)
    print(c_y)
    time = []
    x = []
    x_v = []
    x_a = []
    y = []
    for t in range(0, T+1):
        time.append(t)
        x.append(s(c, t)[0, 0])
        x_v.append(d_s(c, t)[0, 0])
        x_a.append(dd_s(c, t)[0, 0])
        y.append(s(c_y, t)[0, 0])
        # print(s(c, t))
    print(x)
    print(y)
    plt.rcParams['font.sans-serif'] = ['SimHei']
    gs = gridspec.GridSpec(30, 80)
    ax1 = plt.subplot(gs[1:9, 1:30])
    ax1.grid()
    ax1.set_ylabel('x方向位置')
    # ax1.set_yticks(np.arange(1, max(x)))
    plt.plot(time, x, 'b')
    ax2 = plt.subplot(gs[11:19, 1:30])
    ax2.grid()
    ax2.set_ylabel('x方向速度')
    plt.plot(time, x_v, 'r')
    ax3 = plt.subplot(gs[21:30, 1:30])
    ax3.set_ylabel('x方向加速度')
    plt.plot(time, x_a, 'g')
    ax3.grid()
    ax4 = plt.subplot(gs[1:30, 31:80])
    waypoints_x = [x[0] for x in waypoints]
    waypoints_y = [x[1] for x in waypoints]
    ax4.plot(waypoints_x, waypoints_y, 'k')
    ax4.plot(x, y, 'c')
    # ax1 = plt.subplot(3, 2, 1)
    # ax1.grid()
    # plt.plot(time, x, 'b')
    # ax2 = plt.subplot(3, 2, 2)
    # ax2.grid()
    # plt.plot(time, x_v, 'r')
    # ax3 = plt.subplot(3, 2, 3)
    # plt.plot(time, x_a, 'g')
    # ax3.grid()
    plt.show()


if __name__ == '__main__':
    waypoints = []
    max_range = 20
    for i in  range(0, 10):
        waypoints.append([np.random.randint(0, max_range), np.random.randint(0, max_range)])
    boundary_cond = [waypoints[0][0], waypoints[len(waypoints)-1][0], 0, 1, 0, 0]
    boundary_cond_y = [waypoints[0][1], waypoints[len(waypoints)-1][1], 0, 1, 0, 0]
    T = 20
    print(boundary_cond[0], boundary_cond_y[0])
    print(boundary_cond[1], boundary_cond_y[1])
    traj_plan(waypoints, boundary_cond, boundary_cond_y, T)
