import numpy as np
import numpy.linalg as LA
from scipy.special import binom
import math
import matplotlib.pyplot as plt

# Calculate the virtual force exerted by the original trajectory on the "follower."
def calculate_mov_direction(b3,b2,b1,obj_point,f1,f2,f3,kat1,kat2,kat3,kre1,kre2,kre3):
    # Magnitude of attraction to the next point
    att1 = (np.array(f1) - np.array(obj_point))
    # Magnitude of attraction to the next two points
    att2 = (np.array(f2) - np.array(obj_point))
    # Magnitude of attraction to the next three points
    att3 = (np.array(f3) - np.array(obj_point))
    # Magnitude of repulsion from the previous point
    req1 = (np.array(obj_point) - np.array(b1))
    # Magnitude of repulsion from the previous two points
    req2 = (np.array(obj_point) - np.array(b2))
    # Magnitude of repulsion from the previous three points
    req3 = (np.array(obj_point) - np.array(b3))
    ## Combine attraction and repulsion
    return kat1 * np.array(att1) + kre1 * np.array(req1) + kat2 * np.array(att2) + kre2 * np.array(req2) + kat3 * np.array(att3) + kre3 * np.array(req3)

def calculate_mov_direction_total(b3, b2, b1, obj_point, f1, f2, f3, kat1, kat2, kat3, kre1, kre2, kre3,obstacles,kre,km):
    #没有障碍物时的合力
    force_origin = np.array(calculate_mov_direction(b3,b2,b1,obj_point,f1,f2,f3,kat1,kat2,kat3,kre1,kre2,kre3))

    end_point = f1
    req_total = np.array((0,0))
    for obstacle in obstacles:
        x_obstacle = obstacle['x']  # Get x-coordinate of the obstacle's center
        y_obstacle = obstacle['y']  # Get y-coordinate of the obstacle's center
        r_obstacle = obstacle['r']  # Get radius of the obstacle
        p_obstacle = obstacle['p']  # Get influence range of the obstacle
        obstacle_point = (x_obstacle,y_obstacle)
        intersection = intersect_line_circle(obstacle_point, r_obstacle, obj_point, end_point)
        if intersection is not None:

            dis = np.linalg.norm(np.array(obj_point) - np.array(intersection))


            if dis < p_obstacle:
                # Magnitude of attraction.
                att_dis = np.linalg.norm(np.array(end_point) - np.array(obj_point))  # 吸引力大小
                # Magnitude of repulsion.
                req_dis = np.linalg.norm(np.array(obj_point) - np.array(obstacle_point))
                max = att_dis * km#实验1和2是1.1,实验2是1.8
                # The maximum value of repulsion is how many times the attraction, to prevent sudden changes.
                req_p = - max / p_obstacle * dis + max
                # The appropriate magnitude of repulsion.
                req = req_p/req_dis * (np.array(obj_point) - np.array(obstacle_point))

                # The final repulsive force.
            else:
                req = (0,0)
                # The final repulsive force.
        else:
            req = (0, 0)
            # The final repulsive force.

        req_total = req_total + np.array(req)
    return force_origin + kre * np.array(req_total)
    # Combine attraction and repulsion.

# 计算曲率的函数
def compute_curvature(x,y):
    if (x[1] == x[0] and y[1] == y[0]) or (x[1] == x[2] and y[1] == y[2]) or (x[0] == x[2] and y[0] == y[2]):
        return 0  # 如果有重复值，直接返回0

    t_a = LA.norm([x[1] - x[0], y[1] - y[0]])
    t_b = LA.norm([x[2] - x[1], y[2] - y[1]])

    M = np.array([
        [1, -t_a, t_a ** 2],
        [1, 0, 0],
        [1, t_b, t_b ** 2]
    ])

    a = np.matmul(LA.inv(M), x)
    b = np.matmul(LA.inv(M), y)

    kappa = abs(2 * (a[2] * b[1] - b[2] * a[1]) / (a[1] ** 2. + b[1] ** 2.) ** (1.5))
    return kappa


def frechet_distance(trajectory1, trajectory2):
    n = len(trajectory1)
    m = len(trajectory2)

    # 计算两点之间的距离函数
    def distance(pt1, pt2):
        return np.linalg.norm(pt1 - pt2)

    # 初始化距离矩阵
    dp = np.zeros((n, m))
    for i in range(n):
        for j in range(m):
            dp[i, j] = -1

    # 递归计算弗雷歇距离
    def compute_distance(i, j):
        if dp[i, j] != -1:
            return dp[i, j]

        if i == 0 and j == 0:
            dp[i, j] = distance(trajectory1[0], trajectory2[0])
        elif i > 0 and j == 0:
            dp[i, j] = max(compute_distance(i - 1, 0), distance(trajectory1[i], trajectory2[0]))
        elif i == 0 and j > 0:
            dp[i, j] = max(compute_distance(0, j - 1), distance(trajectory1[0], trajectory2[j]))
        elif i > 0 and j > 0:
            dp[i, j] = max(min(compute_distance(i - 1, j), compute_distance(i - 1, j - 1), compute_distance(i, j - 1)),
                           distance(trajectory1[i], trajectory2[j]))
        return dp[i, j]

    return compute_distance(n - 1, m - 1)

# 定义节点类
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

# 定义RRT类
class RRT:
    def __init__(self, start, goal, obstacles, step_size=0.5, max_iter=1000, threshold=1.0):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iter = max_iter
        self.threshold = threshold
        self.nodes = [self.start]

    # 检查节点是否在障碍物内
    def in_obstacle(self, node):
        for obstacle in self.obstacles:
            if np.linalg.norm([node.x - obstacle[0], node.y - obstacle[1]]) <= obstacle[2]:
                return True
        return False

    # 执行RRT算法
    def plan(self):
        for _ in range(self.max_iter):
            # 随机采样一个点
            rand_node = Node(np.random.randint(0, 11), np.random.randint(0, 11))
            # 找到距离随机点最近的节点
            nearest_node = min(self.nodes, key=lambda n: np.linalg.norm([n.x - rand_node.x, n.y - rand_node.y]))
            # 计算向量并规范化
            direction = np.array([rand_node.x - nearest_node.x, rand_node.y - nearest_node.y])
            norm = np.linalg.norm(direction)
            direction = direction / norm if norm != 0 else direction
            # 创建新节点
            new_node = Node(nearest_node.x + self.step_size * direction[0], nearest_node.y + self.step_size * direction[1])
            # 如果新节点在障碍物内，跳过该节点
            if self.in_obstacle(new_node):
                continue
            # 连接新节点和最近节点
            new_node.parent = nearest_node
            self.nodes.append(new_node)
            # 如果新节点接近目标节点，停止搜索
            if np.linalg.norm([new_node.x - self.goal.x, new_node.y - self.goal.y]) <= self.threshold:
                self.goal.parent = new_node
                break

    # 从目标节点反向搜索路径
    def find_path(self):
        path = []
        current = self.goal
        while current is not None:
            path.append([current.x, current.y])
            current = current.parent
        return path[::-1]

# 计算贝塞尔曲线点
def bezier_curve(points, num_points=100):
    n = len(points) - 1
    curve = []
    for t in np.linspace(0, 1, num_points):
        x, y = 0, 0
        for i, point in enumerate(points):
            x += binom(n, i) * ((1 - t) ** (n - i)) * (t ** i) * point[0]
            y += binom(n, i) * ((1 - t) ** (n - i)) * (t ** i) * point[1]
        curve.append([x, y])
    return curve

# 绘制圆和直线，并显示交点
# Draw a circle and a line, then display their intersection points.
def plot_circle_and_line(center, radius, point1, point2, intersection1):
    # 创建一个新的图形
    # Create a new fig.
    fig, ax = plt.subplots()

    # 绘制圆
    # Draw a circle.
    circle = plt.Circle(center, radius, fill=False)
    ax.add_artist(circle)

    # 绘制直线
    # Draw a line.
    if point1[0] != point2[0]:
        x = np.linspace(-10, 10, 100)
        y = (point2[1] - point1[1]) / (point2[0] - point1[0]) * (x - point1[0]) + point1[1]
        ax.plot(x, y)
    else:
        ax.axvline(x=point1[0], linestyle='--')

    # 绘制 A 点和 B 点
    # Draw points A and B.
    ax.plot(point1[0], point1[1], 'ro', label='Point A')
    ax.plot(point2[0], point2[1], 'bo', label='Point B')
    ax.plot(intersection1[0], intersection1[1], 'ko', label='The intersection point')
    ax.set_aspect('equal', 'box')
    # 设置坐标轴范围
    # Set the range of the coordinate axes.
    ax.set_xlim(-10, 20)
    ax.set_ylim(-10, 20)

    # 添加图例
    # Add legend
    ax.legend(loc='upper right', bbox_to_anchor=(1, 1))

    plt.show()


def intersect_line_circle(center, radius, pointA, pointB):
    # Calculate the slope and intercept of the line
    # Check if the line is parallel to the y-axis
    if pointA[0] == pointB[0]:
        slope = 10000  # Infinite slope
    else:
        # Calculate the slope and intercept of the line
        slope = (pointB[1] - pointA[1]) / (pointB[0] - pointA[0])
    intercept = pointA[1] - slope * pointA[0]

    # Use quadratic equation to find the coordinates of the intersection points
    a = 1 + slope ** 2
    b = -2 * center[0] + 2 * slope * intercept - 2 * center[1] * slope
    c = center[0] ** 2 + intercept ** 2 - 2 * center[1] * intercept + center[1] ** 2 - radius ** 2

    discriminant = b ** 2 - 4 * a * c  # Discriminant

    # If discriminant is less than 0, there are no intersection points
    if discriminant < 0:
        # print('No intersection points')
        point1 = None
        point2 = None
    # If discriminant equals 0, there is one intersection point
    elif discriminant == 0:
        x = -b / (2 * a)
        y = slope * x + intercept

        point1 = [x, y]
        point2 = None
    # If discriminant is greater than 0, there are two intersection points
    else:
        x1 = (-b + math.sqrt(discriminant)) / (2 * a)
        y1 = slope * x1 + intercept
        x2 = (-b - math.sqrt(discriminant)) / (2 * a)
        y2 = slope * x2 + intercept

        point1 = [x1, y1]
        point2 = [x2, y2]

    # Calculate the intersection point closest to point A
    if point1 is not None and point2 is not None:
        distance1 = math.sqrt((point1[0] - pointA[0]) ** 2 + (point1[1] - pointA[1]) ** 2)
        distance2 = math.sqrt((point2[0] - pointA[0]) ** 2 + (point2[1] - pointA[1]) ** 2)

        if distance1 < distance2:
            return point1
            # distanceA1 = math.sqrt((point1[0] - pointA[0]) ** 2 + (point1[1] - pointA[1]) ** 2)
            # distanceB1 = math.sqrt((point1[0] - pointB[0]) ** 2 + (point1[1] - pointB[1]) ** 2)
            # if distanceB1 < distanceA1 : #close to obstacle
            #     return point1
            # else:
            #     return None
        else:
            return point2
            # distanceA1 = math.sqrt((point2[0] - pointA[0]) ** 2 + (point2[1] - pointA[1]) ** 2)
            # distanceB1 = math.sqrt((point2[0] - pointB[0]) ** 2 + (point2[1] - pointB[1]) ** 2)
            # if distanceB1 < distanceA1  or np.linalg.norm(pointB - center) < radius:  # close to obstacle
            #     return point2
            # else:
            #     return None

    elif point1 is not None:
        return point1
        # distanceA1 = math.sqrt((point1[0] - pointA[0]) ** 2 + (point1[1] - pointA[1]) ** 2)
        # distanceB1 = math.sqrt((point1[0] - pointB[0]) ** 2 + (point1[1] - pointB[1]) ** 2)
        # if distanceB1 < distanceA1:  # close to obstacle
        #     return point1
        # else:
        #     return None

    elif point2 is not None:
        return point2
        # distanceA1 = math.sqrt((point2[0] - pointA[0]) ** 2 + (point2[1] - pointA[1]) ** 2)
        # distanceB1 = math.sqrt((point2[0] - pointB[0]) ** 2 + (point2[1] - pointB[1]) ** 2)
        # if distanceB1 < distanceA1:  # close to obstacle
        #     return point2
        # else:
        #     return None

    # If there are no intersection points, return None
    else:
        return None


# def compute_curvature(x, y):
#     dx_dt = np.gradient(x)
#     dy_dt = np.gradient(y)
#     dx_dt2 = np.gradient(dx_dt)
#     dy_dt2 = np.gradient(dy_dt)
#
#     denominator = (dx_dt ** 2 + dy_dt ** 2) ** 1.5
#     denominator[denominator == 0] = np.nan  # 将分母为零的元素替换为 NaN，避免除以零的情况
#
#     curvature = np.abs(dx_dt2 * dy_dt - dx_dt * dy_dt2) / denominator
#     return curvature

