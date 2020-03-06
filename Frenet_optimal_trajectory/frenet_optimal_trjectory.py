# 2019-01-07
# Frenet optimal trajectory generator
# reference: [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import cubic_spline_planner

# Parameter
MAX_SPEED = 50.0 / 3.6           # maximum speed [m/s]
MAX_ACCEL = 2.0                  # maximum acceleration [m/ss]
MAX_CURVATURE = 1.0              # 最大曲率 [1/m]
MAX_ROAD_WIDTH = 7.0             # maximum road width [m]
D_ROAD_W = 1.0                   # 道路宽度采样间隔 [m]
DT = 0.5                         # Delta T [s]
MAXT = 5.0                       # 最大预测时间 [s]
MINT = 4.0                       # 最小预测时间 [s]
TARGET_SPEED = 30.0 / 3.6        # 即纵向的速度保持 [m/s]
D_T_S = 5.0 / 3.6                # 目标速度采样间隔 [m/s]
N_S_SAMPLE = 1                   # 目标速度的采样数量
ROBOT_RADIUS = 2.0               # robot radius [m]

# 损失函数权重
KJ = 0.1        # Jerk
KT = 0.1        # 轨迹时长
KD = 1.0        # 偏离距离
KLAT = 1.0      # lateral
KLON = 1.0      # longitudinal

show_animation = True

# 五次多项式
class quintic_polynomial:
    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
        # 计算五次多项式系数
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T**3, T**4, T**5],[3 * T ** 2, 4 * T ** 3, 5 * T ** 4],[6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,  vxe - self.a1 - 2 * self.a2 * T,  axe - 2 * self.a2])
        x = np.linalg.solve(A, b) # 使用Numpy中的 np.linalg.solve(A, b) 方法将矩阵解了出来

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5
        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4
        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3
        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2
        return xt


# 四次多项式
class quartic_polynomial:
    def __init__(self, xs, vxs, axs, vxe, axe, T):
        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * T ** 2, 4 * T ** 3],[6 * T, 12 * T ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,axe - 2 * self.a2])
        x = np.linalg.solve(A, b)
        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4
        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3
        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2
        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t
        return xt


# 使用基于Frenet的优化轨迹方法生成一系列横向和纵向的轨迹，并且计算每条轨迹对应的损失.
class Frenet_path:
    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []
    # 横向动作规划
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):   # sampling in the road width direction
        for Ti in np.arange(MINT, MAXT, DT):   # sampling in time period [4,5,0.2]
            fp = Frenet_path()  # class Frenet_path
            # 计算出关于目标配置di，Ti的横向多项式
            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)  # class quintic_polynomial
            fp.t = [t for t in np.arange(0.0, Ti, DT)]  
            fp.d = [lat_qp.calc_point(t) for t in fp.t]    # calculate the d value by a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]   
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # 纵向速度规划 (速度保持)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S): # speed sampling
                tfp = copy.deepcopy(fp)
                lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)  # class quartic_polynomial

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1])**2
                # the cost calculation
                # 横向的损失函数
                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1]**2   # class Frenet_path
                # 纵向的损失函数
                tfp.cv = KJ * Js + KT * Ti + KD * ds
                # 总的损失函数为d 和 s方向的损失函数乘对应的系数相加
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv
                frenet_paths.append(tfp)
    return frenet_paths


def calc_global_paths(fplist, csp):
    for fp in fplist:
        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])   # Frenet Coordinate transfer to x-y
            if ix is None:
                break
            iyaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)
        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.sqrt(dx**2 + dy**2))
        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])
    return fplist


def check_collision(fp, ob):
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0])**2 + (iy - ob[i, 1])**2)   for (ix, iy) in zip(fp.x, fp.y)]
        collision = any([di <= ROBOT_RADIUS**2 for di in d])
        if collision:
            return False
    return True

# 由于我们将障碍物规避问题都简化为静态了，所以在这里我们只简单地计算了所有规划点到障碍物的距离，一句距离预计是否会发生碰撞，来看看完整的优化轨迹检查函数
def check_paths(fplist, ob):
    okind = []
    for i in range(len(fplist)):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue
        okind.append(i)
    return [fplist[i] for i in okind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, ob)
    # find minimum cost path
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp
    return bestpath


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)
    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))
    return rx, ry, ryaw, rk, csp


def main():
    print(__file__ + " start!!")
    # way points 路线
    wx = [0.0, 10.0, 20.5, 35.0, 70.5]
    wy = [0.0, -6.0, 5.0, 6.5, 0.0]

    # obstacle lists 障碍物列表 
    ob = np.array( [ [20.0, 10.0],[30.0, 6.0],[30.0, 8.0],[35.0, 8.0],[50.0, 3.0] ] )
    # 生成如下参考路径以及障碍物：
    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy) #csp = cubic_spline_planner.Spline2D(x, y), generate 2D spline from waypoints
    # 其中红线就是我们的全局路径，蓝点为障碍物
    plt.plot(wx,wy,'bo')
    plt.plot(tx,ty)
    plt.show()

    # initial state
    c_speed = 10.0 / 3.6    # current speed [m/s]
    c_d = 2.0               # current lateral position [m]
    c_d_d = 0.0             # current lateral speed [m/s]
    c_d_dd = 0.0            # current latral acceleration [m/s]
    s0 = 0.0                # current course position
    area = 20.0             # animation area length [m]

    for i in range(500):
        # the main loop, calculate for every prediction time period

        # path derived from Frenrt_path class
        path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)

        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]
        # sqrt(x*x + y*y) determine if the first path node has reached the goal point
        if np.hypot(path.x[0] - tx[-1], path.y[0] - ty[-1]) <= 1.0: 
            print("Goal")
            break

        # dynamicly plot the trajectory 
        if show_animation:
            plt.cla()
            plt.plot(tx, ty)   # the spline points from waypoints
            plt.plot(ob[:, 0], ob[:, 1], "xk")  # obstacles
            plt.plot(path.x[0:], path.y[0:], "-or")
            plt.plot(path.x[0], path.y[0], "vc")
            plt.xlim(path.x[0] - area, path.x[0] + area)
            plt.ylim(path.y[0] - area, path.y[0] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)

    print("Finish")
    if show_animation:
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()

if __name__ == '__main__':
 main()
