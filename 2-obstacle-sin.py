import numpy as np
import matplotlib.pyplot as plt

from basic import calculate_mov_direction_total, calculate_mov_direction,intersect_line_circle


#设置部分参数
# Set initial parameters
kre = 1
kat1 = 1
kat2 = 0.1
kat3 = 0.1
kre1 = 0.1
kre2 = 0.01
kre3 = 0.01
step_max = 0.11
disErr = 0.2

# 创建10x10的网格 Create a 10x10 Grid
X, Y = np.meshgrid(np.arange(0, 11), np.arange(0, 11))
plt.plot(X, Y, 'k-')
plt.plot(Y, X, 'k-')
plt.xticks(fontsize=14, weight='bold')
plt.yticks(fontsize=14, weight='bold')
plt.xlabel('X', fontsize=14, weight='bold')  # 设置X轴标签
plt.ylabel('Y', fontsize=14, weight='bold')  # 设置Y轴标签
#生成轨迹 Generate Trajectory
x = np.linspace(0, 10, 150)  # 在0到10之间生成等间隔的200个点
y = 5*np.sin(x)+5  # 计算正弦值

Traj = np.column_stack((x, y))  # 将x和y合并为一个二维数组
#Real-time smooth obstacle avoidance with a straight path and only one obstacle.


#添加障碍物 # Add obstacles
obstacle1 = {'x': 2, 'y': 3.5, 'r': 1, 'p': 2.5}
obstacle2 = {'x': 6.8, 'y': 5, 'r': 1, 'p': 2.5}

obstacles = [obstacle1,obstacle2]
obs_index = obstacle1
# 绘制起点和终点 # Plot start and goal points
start = Traj[0]
goal = Traj[-1]

plt.plot(start[0], start[1], 'bo', markersize=10)
plt.plot(goal[0], goal[1], 'ro', markersize=10)

#画出障碍物 # Plot obstacles
for obstacle in obstacles:
    x_obstacle = obstacle['x']  # Get x-coordinate of the obstacle's center
    y_obstacle = obstacle['y']  # Get y-coordinate of the obstacle's center
    r_obstacle = obstacle['r']  # Get radius of the obstacle
    p_obstacle = obstacle['p']  # Get influence range of the obstacle

    circle1 = plt.Circle((x_obstacle,y_obstacle), radius=p_obstacle, color='gray')
    plt.gca().add_artist(circle1)

    circle2 = plt.Circle((x_obstacle,y_obstacle), radius=r_obstacle, color='black')
    plt.gca().add_artist(circle2)

# 初始化移动点的位置和轨迹
# Initialize position of moving point and trajectory
position = np.array(start)
trajectory = [position]
# 设置坐标轴范围和标签
# Set axis range and labels
plt.xlim([-1, 11])
plt.ylim([-1, 11])
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Map')

#设置平滑轨迹的起点和集合 Set the starting point and collection of the smoothed trajectory.
obj_point = start
follow = [obj_point]

i=0
count =0


pos_obj_last=10
# 移动点到达终点前不断运动
# Move point until it reaches the goal
while np.linalg.norm(obj_point - goal) > 0.15:
    # Calculate the next position of the moving point
    if i < Traj.shape[0] - 2 and i > 3:
        # Define the points for calculation
        b3 = (Traj[i - 3][0], Traj[i - 3][1])
        b2 = (Traj[i - 2][0], Traj[i - 2][1])
        b1 = (Traj[i - 1][0], Traj[i - 1][1])
        f1 = (Traj[i][0], Traj[i][1])
        f3 = (Traj[i + 2][0], Traj[i + 2][1])
        f2 = (Traj[i + 1][0], Traj[i + 1][1])

    elif i < Traj.shape[0]:
        # Define the points for calculation
        b3 = (Traj[i][0], Traj[i][1])
        b2 = (Traj[i][0], Traj[i][1])
        b1 = (Traj[i][0], Traj[i][1])
        f1 = (Traj[i][0], Traj[i][1])
        f3 = (Traj[i][0], Traj[i][1])
        f2 = (Traj[i][0], Traj[i][1])

    if i < 3:  # The first three points only provide attraction force
        delta = calculate_mov_direction(b3, b2, b1, obj_point, f1, f2, f3, kat1, 0, 0, 0, 0, 0)
    elif i > Traj.shape[0] - 2:  # The last three points only provide attraction force
        delta = calculate_mov_direction(b3, b2, b1, obj_point, f1, f2, f3, kat1, 0, 0, 0, 0, 0)
    else:  # Normal calculation for intermediate points
        delta = calculate_mov_direction_total(b3, b2, b1, obj_point, f1, f2, f3, kat1, kat2, kat3, kre1, kre2, kre3,obstacles,kre,1.1)

    # 计算下一个原始轨迹点的位置
    #Calculate the position of the next original trajectory point.
    if i<Traj.shape[0]:
        next_position = (Traj[i][0], Traj[i][1])
    #对步长进行限制
    length = np.linalg.norm(delta)
    if length > step_max:
        delta = step_max / length * delta

    # 计算下一个平滑轨迹点的位置 Calculate the position of the next smoothed trajectory point.
    obj_point = obj_point + delta
    follow.append(obj_point)
    # 更新位置并记录轨迹 # Update position and record trajectory
    position = np.array(next_position)
    trajectory.append(position)

    for obstacle in obstacles:
        x_obstacle = obstacle['x']  # Get x-coordinate of the obstacle's center
        y_obstacle = obstacle['y']  # Get y-coordinate of the obstacle's center
        r_obstacle = obstacle['r']  # Get radius of the obstacle
        p_obstacle = obstacle['p']  # Get influence range of the obstacle
        obstacle_point = (x_obstacle, y_obstacle)
        if np.linalg.norm(position - obstacle_point) > p_obstacle:
            continue
        else:
            obs_index = obstacle
            break
    #设置吸引子的移动速度 Setting the moving speed of the attractor.
    # for obstacle in obstacles:
    x_obstacle = obs_index['x']  # Get x-coordinate of the obstacle's center
    y_obstacle = obs_index['y']  # Get y-coordinate of the obstacle's center
    r_obstacle = obs_index['r']  # Get radius of the obstacle
    p_obstacle = obs_index['p']  # Get influence range of the obstacle
    obstacle_point = (x_obstacle,y_obstacle)
    print(p_obstacle)
    if np.linalg.norm(position - obstacle_point) > p_obstacle:
        if np.linalg.norm(obj_point - position) < disErr:
            i = i + 1
        else:
            if count % 4 == 0:
                i = i + 1

    else:#更加精细地调节速度，会取得更好的结果，可以进行多次尝试
        ## Fine-tuning the speed more precisely will yield better results. Multiple attempts can be made for optimization.
        intersection = intersect_line_circle(obstacle_point, r_obstacle, obj_point, position)
        if intersection is not  None:
            pos_obj = np.linalg.norm(obstacle_point - position)
            ky=0.4
            if pos_obj <= pos_obj_last and pos_obj >= r_obstacle+ky:# 离障碍物越来越近,速度加快 # Speed increases as the obstacle gets closer
                i = i + 2
            elif pos_obj <= pos_obj_last and pos_obj < r_obstacle+ky:
                if count % 6 == 0:
                    i = i + 1
            elif pos_obj > pos_obj_last and pos_obj < r_obstacle+ky:
                if count % 2 == 0:
                    i = i + 1
            elif pos_obj > pos_obj_last and pos_obj >= r_obstacle+ky:
                if count % 1 == 0:
                    i = i + 1
                    #kre=0
            pos_obj_last = pos_obj
        else:
            if np.linalg.norm(obj_point - position) < disErr:
                i = i + 1
            else:
                if count % 3 == 0:
                    i = i + 1

    trajectory1 = np.array(trajectory)
    plt.plot(trajectory1[:, 0], trajectory1[:, 1], 'yo-', lw=2, markersize=3, label='Original trajectory')
    follow1 = np.array(follow)
    plt.plot(follow1[:, 0], follow1[:, 1], 'bo-', lw=2, markersize=3, label='Smoothed trajectory')
    plt.draw()
    plt.pause(0.1)

    if count == 0:
        plt.legend(loc='upper right',fontsize=14)  # Add legend and specify position to upper right

    count =count + 1

plt.show()
