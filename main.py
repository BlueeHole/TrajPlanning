'''
【开发计划】

1. 路径搜索 & 仿真简易平台搭建  【8.12前】
    （1）实现读入图片转化为二维数组 √
    （2）实现在此基础上的bfs算法并可视化 √ (注意窗口是Esc退出)
    （3）实现A* √
2. 路径平滑
3. 速度规划【8.16前】
    （1）实现速度规划
    （2）实现速度规划可视化（可用Matplotlib）
    （*）可以的话最好实现一辆车的控制仿真
4. 路径跟随
'''


from queue import Queue, PriorityQueue
from map_manager import Map
from threading import Thread
import operator
import math
import cv2 as cv
import time

BLOCK_SIZE = 5
REACH_THREHOLD = BLOCK_SIZE
start = (4, 4)
goal = (150, 300)


def dis(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def calcCost(now, nxt):
    return dis(now, nxt)


def heuristic(a, b):
   # Manhattan distance on a square grid
   return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star(map, start, goal):
    path_list = []
    from_where = dict()
    visited = set()
    from_where[start] = (0, 0)
    expand = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    q = PriorityQueue()
    q.put(start, heuristic(start, goal))
    cost_so_far = dict()
    cost_so_far[start] = 0
    visited.add(start)
    reached = False

    if operator.eq(map.getMap()[goal[0]][goal[1]], 255):
        while not q.empty():
            now = q.get()
            # print(now)
            if dis(now, goal) < REACH_THREHOLD:
                reached = True
                print('找到路径')
                p = now
                # 这种写法不会把起始点加入进去
                while from_where[p] != (0, 0):
                    path_list.append(p)
                    p = from_where[p]
                break
            visited.add(now)
            for exp in expand:
                nxt = (now[0] + BLOCK_SIZE*exp[0], now[1] + BLOCK_SIZE*exp[1])
                nxt_cost = cost_so_far[now] + calcCost(now, nxt)
                if nxt in cost_so_far:
                    if nxt_cost >= cost_so_far[nxt]:
                        continue
                if not map.inMap(nxt):
                    continue
                if nxt in visited:
                    continue
                # TODO: 只有纯白代表没障碍
                if not operator.eq(map.getMap()[nxt[0]][nxt[1]], 255):
                    continue
                # visited.add(nxt)
                cost_so_far[nxt] = nxt_cost
                q.put(nxt, nxt_cost + heuristic(nxt, goal))
                from_where[nxt] = now

    if not reached:
        print('未找到路径')
    return path_list


def velocity_planning(search_map, path_list):
    pass


def traj_planning(search_map, start, new_goal):
    start_time = time.time()
    path = a_star(search_map, start, new_goal)
    end_time = time.time()
    print('[路径搜索]耗时{:.4f}秒'.format((end_time-start_time)))
    return path


def visualize_path(path, start, goal):
    # 要求：path包含终点而不包含起点
    rm_map.refresh()
    rm_map.getMap()[start[0]: start[0] + BLOCK_SIZE, start[1]: start[1] + BLOCK_SIZE] = [0, 0, 255]
    rm_map.getMap()[goal[0]: goal[0] + BLOCK_SIZE, goal[1]: goal[1] + BLOCK_SIZE] = [0, 255, 0]
    x = 255
    for p in path[1:]:  # 不包括终点
        x = (x - 10) % 256  # 变色绘制路径
        rm_map.getMap()[p[0]: p[0] + BLOCK_SIZE, p[1]: p[1] + BLOCK_SIZE] = [x, x, 0]


def setGoal(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        # print('{}, {}'.format(x, y))
        # 注意这里有个x, y是反着的关系
        goal_x = y//5 * 5 - 1
        goal_y = x//5 * 5 - 1
        new_goal = (goal_x, goal_y)
        print(new_goal)
        path = traj_planning(search_map, start, new_goal)
        print(len(path))

        path_ = [path[x] for x in range(0, len(path), 10)]

        from mutli_seg_traj_generator import traj_generator, visualize_traj
        start_time = time.time()
        traj_list = traj_generator(path_)
        end_time = time.time()
        print('[轨迹生成]耗时{:.4f}秒'.format((end_time - start_time)))

        visualize_traj(path_, traj_list)
        # visualize_path(path, start, new_goal)


if __name__ == '__main__':
    rm_map = Map('map.jpg')
    # try:
    #     t = Thread(target=rm_map.showMap)
    #     t.start()
    # except:
    #     print("Error: 无法启动地图可视化线程")

    gray = cv.cvtColor(rm_map.getMap(), cv.COLOR_BGR2GRAY)
    retval, map = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)
    search_map = Map('map.jpg')     # 随便初始化一下，然后把它设为实际上是灰度图
    search_map.setMap(map)

    while True:
        window = rm_map.showMap()
        cv.setMouseCallback(window, setGoal)
        if cv.waitKey(20) & 0xFF == 27:     # Esc退出
            break
    cv.destroyAllWindows()
