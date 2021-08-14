import numpy as np
from numpy.linalg import solve
import matplotlib.pyplot as plt
from matplotlib import gridspec
import math

plt.rcParams['font.sans-serif'] = ['SimHei']


def getQ(n_seg, n_order, ts):
    Q = np.mat(np.zeros(((n_order+1)*n_seg, (n_order+1)*n_seg)))
    for j in range(0, n_seg):
        Q_j = np.mat(np.zeros((n_order + 1, n_order + 1)))
        # 这里下标特例, 因为公式里i和l太多了不方便改
        for i in range(4, 8):
            for l in range(4, 8):
                Q_j[i, l] = i * (i - 1) * (i - 2) * (i - 3) \
                                    * l * (l - 1) * (l - 2) * (l - 3) / (i + l - 7)\
                                    * math.pow(ts[j], i + l - 7)
            # 对角矩阵拼接
        Q[j*(n_order+1):(j+1)*(n_order+1), j*(n_order+1):(j+1)*(n_order+1)] = Q_j
    return Q

def getM(n_seg, n_order, ts):
    M = np.mat(np.zeros((8*n_seg, (n_order+1)*n_seg)))
    for k in range(0, n_seg):
        M_k = np.mat(np.zeros((8, n_order+1)))
        M_k[0, 0] = 1
        M_k[1, 1] = 1
        M_k[2, 2] = 2
        M_k[3, 3] = 6
        for row in range(4, 8):
            for col in range(row-5, 8):
                M_k[row, col] = math.pow(ts[k], col+1 - (row - 3))
        for row in range(5, 8):
            for col in range(row-5, 8):
                M_k[row, col] = M_k[row - 1, col] * (col - (row - 5))
        M[k*8:(k+1)*8, k*(n_order+1):(k+1)*(n_order+1)] = M_k
    return M

def getCt(n_seg, n_order):
    Ct = np.mat(np.zeros((8 * n_seg, 4 * n_seg + 4)))
    Ct[0, 0] = 1
    Ct[1, 1] = 1
    Ct[2, 2] = 1
    Ct[3, 3] = 1
    row = 4
    for i in range(4, 5+(n_seg-1)-1):
        Ct[row, i] = 1
        Ct[row + 4, i] = 1
        row = row + 8
    Ct[8 * n_seg - 4, n_seg + 3] = 1
    Ct[8 * n_seg - 3, n_seg + 4] = 1
    Ct[8 * n_seg - 2, n_seg + 5] = 1
    Ct[8 * n_seg - 1, n_seg + 6] = 1
    for offset in range(5, 8):
        row = offset
        for i in range(n_seg+8+(offset-6), 4*n_seg+2+(offset-6)+1, 3):
            Ct[row, i] = 1
            Ct[row+4, i] = 1
            row = row + 8
    return Ct

def MinimumSnapCloseformSolver(waypoints, ts, n_seg, n_order):
    # waypoints
    start_cond = [waypoints[0], 0, 0, 0]
    end_cond =   [waypoints[-1], 0, 0, 0]
    ######################################################
    # you have already finished this function in hw1
    Q = getQ(n_seg, n_order, ts)
    ######################################################
    # STEP 1: compute M
    M = getM(n_seg, n_order, ts)
    ######################################################
    # STEP 2: compute Ct
    Ct = getCt(n_seg, n_order)
    C = Ct.T
    import pandas as pd
    pdr = pd.DataFrame(C)
    pdr.to_excel('C.xlsx', index=False, header=False)

    R = C * M.I.T * Q * M.I * Ct

    # import pandas as pd
    # pd = pd.DataFrame(C)
    # pd.to_excel('C.xlsx', index=False, header=False)

    # print(R.shape)
    R_fp = R[0:n_seg+7, n_seg+7:]
    R_pp = R[n_seg+7:, n_seg+7:]
    ######################################################
    # STEP 3: compute dF
    dF = np.mat(np.zeros((n_seg+7, 1)))   # 列向量

    for i in range(0, 4):
        dF[i, 0] = start_cond[i]
        dF[n_seg+3+i, 0] = end_cond[i]

    for i in range(4, 5+(n_seg-1)-1):
        dF[i, 0] = waypoints[i-3]
    # dF = dF.T   # 行向量变列向量
    # print(R_pp)
    # print(R_pp.shape)
    # print(R_fp.T.shape)
    dP = -R_pp.I * R_fp.T * dF
    poly_coef = M.I * Ct * np.r_[dF, dP]

    return poly_coef


def traj_generator(path_list):
    path = np.mat(np.zeros((len(path_list), 2)))
    for i, point in enumerate(path_list):
        path[i, 0] = point[0]   # x
        path[i, 1] = point[1]   # y
    n_order = 7
    n_seg = len(path_list) - 1
    n_poly_perseg = n_order + 1
    ts = np.mat(np.zeros((n_seg, 1)))
    dist = np.mat(np.zeros((n_seg, 1)))
    dist_sum = 0
    T = 25
    t_sum = 0
    for i in range(0, n_seg):
        dist[i] = math.sqrt((path[i + 1, 0] - path[i, 0])**2 + (path[i + 1, 1] - path[i, 1])**2)
        dist_sum = dist_sum + dist[i]
    for i in range(0, n_seg-1):
        ts[i] = dist[i] / dist_sum * T
        t_sum = t_sum + ts[i]
    ts[n_seg-1] = T - t_sum
    #TODO: 时间缩放
    for i in range(0, n_seg):
        ts[i] = ts[i] / 5

    poly_coef_x = MinimumSnapCloseformSolver(path[:, 0], ts, n_seg, n_order)
    poly_coef_y = MinimumSnapCloseformSolver(path[:, 1], ts, n_seg, n_order)

    print(poly_coef_x.shape)

    # 取系数和可视化步骤
    X_n = []
    Y_n = []
    tstep = 0.01
    for i in range(0, n_seg):
        Pxi = poly_coef_x[(n_order+1)*i : (n_order+1)*(i+1)]
        Pyi = poly_coef_y[(n_order+1)*i : (n_order+1)*(i+1)]
        Pxi = np.flipud(Pxi)
        Pyi = np.flipud(Pyi)
        print(np.array(Pxi).flatten())
        t = 0
        while t <= ts[i]:
            X_n.append(np.polyval(Pxi, t))
            Y_n.append(np.polyval(Pyi, t))
            t = t + tstep
    plt.plot(X_n, Y_n, 'g-')
    # print(path)
    point_x = np.array(path[:, 0])
    point_y = np.array(path[:, 1])
    # print(point_x.flatten())
    plt.scatter(point_x.flatten(), point_y.flatten())
    plt.show()


if __name__ == '__main__':
    path_list = []
    path_list.append([0.2, 0.2])
    path_list.append([0.3, 0.4])
    path_list.append([0.5, 0.6])
    path_list.append([0.7, 0.8])
    path_list.append([0.7, 0.9])
    path_list.append([1.6, 2.0])
    traj_generator(path_list)