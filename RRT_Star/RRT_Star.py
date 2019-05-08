# https://blog.csdn.net/yuxuan20062007/article/details/88843690
"""
RRT*算法与RRT算法的区别主要在于两个针对新节点 x_{new} 的重计算过程，分别为：
重新为 x_{new} 选择父节点的过程， 比起RRT多了一个rewire的过程。
重布线随机树的过程


重布线过程的意义在于每当生成了新的节点后，是否可以通过重新布线，使得某些节点的路径代价减少。如果以整体的眼光看，并不是每一个重新布线的节点都会出现在最终生成的路径中，但在生成随机树的过程中，每一次的重布线都尽可能的为最终路径代价减小创造机会。

RRT*算法的核心在于上述的两个过程：重新选择父节点和重布线。这两个过程相辅相成，重新选择父节点使新生成的节点路径代价尽可能小，重布线使得生成新节点后的随机树减少冗余通路，减小路径代价。

"""

"""
1. 产生一个随机点xrand。
2. 在树上找到与xrand最近的节点xnearest。
3. 连接xrand与xnearest。
4. 以xrand为中心，ri为半径，在树上搜索节点
5. 找出潜在的父节点集合Xpotential_parent，其目的是要更新xrand，看看有没有比它更好的父节点。
6. 从某一个潜在的父节点xpotential_parent开始考虑。
7. 计算出xparent作为父节点时的代价。
8. 先不进行碰撞检测，而是将xpotential_parent与xchild（也就是xrand）连接起来
9. 计算出这条路径的代价
10. 将新的这条路径的代价与原路径的代价作比较，如果新的这条路径的代价更小则进行碰撞检测，如果新的这条路径代价更大则换为下一个潜在的父节点。
11. 碰撞检测失败，该潜在父节点不作为新的父节点。
12. 开始考虑下一个潜在父节点。
13. 将潜在父节点和xchild连接起来
14. 计算出这条路径的代价。
15. 将新的这条路径的代价与原路径的代价作比较，如果新的这条路径的代价更小则进行碰撞检测，如果新的这条路径代价更大则换为下一个潜在的父节点。
16. 碰撞检测通过。
17. 在树中将之前的边删掉。
18. 在树中将新的边添加进去，将xpotential_parent作为xparent。
19. 遍历所有的潜在父节点，得到更新后的树。

"""


import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt

show_animation = False


class RRT():
    # InitializeTree()
    def __init__(self, start, goal, obstacleList, randArea, expandDis=0.5, goalSampleRate=20, maxIter=1000):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    # the main process
    def Planning(self, animation=True):
        self.nodeList = [self.start]
        # for k=1 to K do:
        for i in range(self.maxIter):
            # x_rand=RANDOM_STATE()
            random_point = self.get_random_point()
            # x_near=NEAREST_NEIGHBOR(x_rand,T)
            nearest_node_index = self.GetNearestListIndex(self.nodeList, random_point)  #nearest node of the random_point
            #x_new=STEER(x_rand,x_near)
            newNode = self.steer(random_point, nearest_node_index)                      #expand tree
            #  print(newNode.cost)
            # if obstaclefree(x_new) then 
            # return x_new
            if self.__CollisionCheck(newNode, self.obstacleList):                   # new_node is not collsion with obstacles 连接xrand与xnearest
                nearinds = self.find_near_nodes(newNode)                            # 以xrand为中心，ri为半径，在树上搜索节点
                newNode = self.choose_parent(newNode, nearinds)                     # 找出潜在的父节点集合Xpotential_parent，其目的是要更新xrand，看看有没有比它更好的父节点
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

            if animation:
                self.DrawGraph(random_point)

        # generate coruse
        lastIndex = self.get_best_last_index()          # 在树中将新的边添加进去，将xpotential_parent作为xparent
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)         # 遍历所有的潜在父节点，得到更新后的树
        return path


##################################################################################################################

    def get_random_point(self):
        if random.randint(0, 100) > self.goalSampleRate:
            random_point = [ random.uniform(self.minrand, self.maxrand), random.uniform(self.minrand, self.maxrand) ]
        else:  # goal point sampling
            random_point = [self.end.x, self.end.y]
        return random_point


    def GetNearestListIndex(self, nodeList, random_point):
        dlist = [ (node.x - random_point[0]) ** 2 + (node.y - random_point[1]) ** 2 for node in nodeList ]
        minearest_node_index = dlist.index( min(dlist) )
        return minearest_node_index

    def steer(self, random_point, nearest_node_index):
        # expand tree
        nearestNode = self.nodeList[nearest_node_index]
        theta = math.atan2(random_point[1] - nearestNode.y, random_point[0] - nearestNode.x)
        newNode = copy.deepcopy(nearestNode)
        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)
        newNode.cost += self.expandDis
        newNode.parent = nearest_node_index
        return newNode

    def __CollisionCheck(self, node, obstacleList):
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # collision
        return True  # safe

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +(node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    # 找出潜在的父节点集合Xpotential_parent，其目的是要更新xrand，看看有没有比它更好的父节点
    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
           return newNode
        dlist = []
        for i in nearinds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minearest_node_index = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minearest_node_index

        return newNode

    # 碰撞检测
    def check_collision_extend(self, nearNode, theta, d):
        tmpNode = copy.deepcopy(nearNode)
        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False
        return True

    # 重布线过程
    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]
            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)
            scost = newNode.cost + d
            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    # 在树中将新的边添加进去，将xpotential_parent作为xparent
    def get_best_last_index(self):
        disglist = [self.calc_dist_to_goal( node.x, node.y ) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]
        #  print(goalinds)
        if len(goalinds) == 0:
            return None
        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i
        return None

    # 遍历所有的潜在父节点，得到更新后的树
    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

#######################################################################################################################


    def DrawGraph(self, random_point=None):
        #Draw Graph
        plt.clf()
        if random_point is not None:
            plt.plot(random_point[0], random_point[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [ node.y, self.nodeList[node.parent].y ], "-g")

        for (ox, oy, size) in self.obstacleList: plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)


class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

########################################################################################################################
def main():
    print("Start rrt planning")
    # the obstacle list
    obstacleList = [ (5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2), (9, 5, 2) ]  # [x,y,size(radius)]
    # Set Initial parameters
    rrt_star = RRT( start=[0, 0], goal=[5, 10], randArea=[-2, 15], obstacleList=obstacleList)
    path = rrt_star.Planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()

    print("Done")


#########################################################################################################################
if __name__ == '__main__':
    main()