#Real-time smooth obstacle avoidance with a straight path and only one obstacle.

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from basic import calculate_mov_direction_total, calculate_mov_direction, compute_curvature, frechet_distance, RRT,bezier_curve

#设置部分参数
# Set initial parameters
kre = 1
kat1 = 1
kat2 = 0.5 #0.1
kat3 = 0
kre1 = 0.01 #0.1
kre2 = 0.01
kre3 = 0
step_max = 0.17 #0.11
disErr = 1  #0.2

# 创建一个新的图形
figsize=(10, 8)
# 创建10x10的网格 # Plot a 10x10 grid
X, Y = np.meshgrid(np.arange(0, 11), np.arange(0, 11))
plt.plot(X, Y, 'k-', linewidth=1)
plt.plot(Y, X, 'k-', linewidth=1)
plt.xticks(fontsize=14, weight='bold')
plt.yticks(fontsize=14, weight='bold')
plt.xlabel('X', fontsize=14, weight='bold')  # 设置X轴标签
plt.ylabel('Y', fontsize=14, weight='bold')  # 设置Y轴标签
#生成轨迹
# 定义V形状的参数
bottom_point = np.array([5, 3])  # V形状的最低点坐标
v_angle = np.deg2rad(45)  # V的角度，转换为弧度
v_length = 5  # V的长度

# 计算V形状的顶部、左侧和右侧的坐标
right_point = bottom_point + np.array([np.cos(v_angle), np.sin(v_angle)]) * v_length
left_point = bottom_point + np.array([-np.cos(v_angle), np.sin(v_angle)]) * v_length

# 在V的左侧和右侧之间均匀插入30个点
num_points_between = 50
left_to_right_vector = right_point - left_point
v_points_between = [left_point + i / (num_points_between + 1) * left_to_right_vector for i in range(1, num_points_between + 1)]

# 采样15个点
sampled_points1_x = np.linspace(left_point[0], bottom_point[0], num_points_between/2)
sampled_points1_y = np.linspace(left_point[1], bottom_point[1], num_points_between/2)
sampled_points1 = np.column_stack((sampled_points1_x, sampled_points1_y))

sampled_points2_x = np.linspace(bottom_point[0], right_point[0], num_points_between/2)
sampled_points2_y = np.linspace(bottom_point[1], right_point[1], num_points_between/2)
sampled_points2 = np.column_stack((sampled_points2_x, sampled_points2_y))

# 将V形状的所有点组合成轨迹
Traj = np.vstack((sampled_points1, sampled_points2))

#添加障碍物 # Add obstacles
obstacle1 = {'x': 5, 'y': 3.5, 'r': 1, 'p': 2.5}


obstacles = [obstacle1]

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


#设置平滑轨迹的起点和集合
obj_point = start
follow = [obj_point]

i=0
count =0

pos_obj_last=10
# 移动点到达终点前不断运动
# Move point until it reaches the goal
while np.linalg.norm(obj_point - goal) > 0.1:
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
        delta = calculate_mov_direction_total(b3, b2, b1, obj_point, f1, f2, f3, kat1, kat2, kat3, kre1, kre2, kre3,obstacles,kre,1.8)

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


    #设置吸引子的移动速度 Setting the moving speed of the attractor.
    for obstacle in obstacles:
        x_obstacle = obstacle['x']  # Get x-coordinate of the obstacle's center
        y_obstacle = obstacle['y']  # Get y-coordinate of the obstacle's center
        r_obstacle = obstacle['r']  # Get radius of the obstacle
        p_obstacle = obstacle['p']  # Get influence range of the obstacle
        obstacle_point = (x_obstacle,y_obstacle)
        if np.linalg.norm(position - obstacle_point) > p_obstacle:
            if np.linalg.norm(obj_point - position) < disErr:
                i = i + 1
                trajectory.append(position)
            else:
                if count % 3 == 0:#4
                    i = i + 1
                    trajectory.append(position)
        else:#更加精细地调节速度，会取得更好的结果，可以进行多次尝试
            ## Fine-tuning the speed more precisely will yield better results. Multiple attempts can be made for optimization.
            pos_obj = np.linalg.norm(obstacle_point - position)
            ky = 0.5
            if pos_obj <= pos_obj_last and pos_obj >= r_obstacle:# 离障碍物越来越近,速度加快 # Speed increases as the obstacle gets closer
                i = i + 2
                trajectory.append(position)
            elif pos_obj <= pos_obj_last and pos_obj < r_obstacle:
                if count % 4 == 0:#5
                    i = i + 1
                    trajectory.append(position)
            elif pos_obj > pos_obj_last and pos_obj <= r_obstacle-0.05:
                if count % 3 == 0:#1
                    i = i + 2#2
                    trajectory.append(position)
            elif pos_obj > pos_obj_last and pos_obj < r_obstacle +ky:
                if count % 1 == 0:  # 1
                    i = i + 5  # 5
                    trajectory.append(position)
            elif pos_obj > pos_obj_last and pos_obj >= r_obstacle+ky:
                if count % 8 == 0:
                    i = i + 1
                    trajectory.append(position)
                    kre=0
            pos_obj_last = pos_obj
            break;


    trajectory1 = np.array(trajectory)
    plt.plot(trajectory1[:, 0], trajectory1[:, 1], 'yo-', lw=2, markersize=3,label='Original trajectory')
    follow1 = np.array(follow)
    plt.plot(follow1[:, 0], follow1[:, 1], 'bo-', lw=2, markersize=3,label='Smoothed trajectory')

    plt.draw()
    plt.pause(0.1)

    if count == 0:
        plt.scatter(trajectory1[0, 0], trajectory1[0, 1], color='blue', marker='o', label='Start')
        plt.scatter(trajectory1[-1, 0], trajectory1[-1, 1], color='red', marker='o', label='Goal')

        plt.legend(loc='upper center', ncol=2, fontsize=14)
    count =count +1
plt.savefig('result/experiment3/apf-90.png', dpi=300)
plt.show()

#
# ##################################################################################################################################################
# # 定义障碍物
# obstacles = [(5, 5.5, 1)]
#
# # 初始化RRT算法
# rrt = RRT(start=(3, 5), goal=(7, 5), obstacles=obstacles)
#
# # 执行路径规划
# rrt.plan()
#
# # 查找路径
# path = rrt.find_path()
#
# # 假设 path 是包含坐标点的列表，每个坐标点有 x 和 y 两个维度
# # 将 path 拆分成 x 和 y 坐标的列表
# x_coords = [point[0] for point in path]
# y_coords = [point[1] for point in path]
#
# # 创建 x 和 y 坐标的插值函数
# f_x = interp1d(np.arange(len(x_coords)), x_coords, kind='linear')
# f_y = interp1d(np.arange(len(y_coords)), y_coords, kind='linear')
#
# # 在新长度范围内进行插值
# new_length = len(x_coords) * 5
# new_indices = np.linspace(0, len(x_coords) - 1, new_length)
# new_x_coords = f_x(new_indices)
# new_y_coords = f_y(new_indices)
#
# # 将新的坐标点组合成新的路径
# path = [(new_x_coords[i], new_y_coords[i]) for i in range(len(new_x_coords))]
#
# # 生成均匀采样的点
# start_point = (0, 5)
# end_point = (3, 5)
# num_points = 15
# x_values = np.linspace(start_point[0], end_point[0], num_points)
# y_values = np.linspace(start_point[1], end_point[1], num_points)
# # 创建包含采样点的数组
# sampled_points = np.column_stack((x_values, y_values))
#
# sampled_points = np.vstack(( sampled_points,path))
#
# # 生成均匀采样的点
# start_point = (7, 5)
# end_point = (10, 5)
# num_points = 15
# x_values = np.linspace(start_point[0], end_point[0], num_points)
# y_values = np.linspace(start_point[1], end_point[1], num_points)
# # 创建包含采样点的数组
# sampled_points1 = np.column_stack((x_values, y_values))
# sampled_points = np.vstack((sampled_points,sampled_points1))
#
# sampled_points_array = np.array(sampled_points)
#
# # 平滑路径
# smoothed_path = bezier_curve(sampled_points_array)
# smoothed_path= np.array(smoothed_path)
# print(len(smoothed_path))
# # 绘制结果
# figsize=(10, 8)
# plt.plot(0, 5, 'bo', markersize=10)
#
#
#
# plt.plot(10, 5, 'ro', markersize=10)
#
# for obstacle in obstacles:
#     circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='black')
#     plt.gca().add_patch(circle)
# for node in rrt.nodes:
#     if node.parent:
#         plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'g-')
# plt.plot([node[0] for node in path], [node[1] for node in path], 'r-', lw=2, markersize=3, label='RRT Path')
# plt.plot([node[0] for node in smoothed_path], [node[1] for node in smoothed_path], 'b-', lw=2, markersize=3, label='Smoothed Path')
#
# plt.scatter(3, 5, color='blue', marker='o', label='Start')
# plt.scatter(7, 5, color='red', marker='o', label='Goal')
#
# plt.legend(loc='upper center', ncol=2,fontsize=14)
#
#
# plt.plot((0,3), (5,5), 'b-')
# plt.plot((7,10), (5,5), 'b-')
#
# X, Y = np.meshgrid(np.arange(0, 11), np.arange(0, 11))
# plt.plot(X, Y, 'k-', linewidth=1)
# plt.plot(Y, X, 'k-', linewidth=1)
# plt.xlim([-1, 11])
# plt.ylim([-1, 11])
# plt.xticks(fontsize=14, weight='bold')
# plt.yticks(fontsize=14, weight='bold')
# plt.xlabel('X', fontsize=14, weight='bold')  # 设置X轴标签
# plt.ylabel('Y', fontsize=14, weight='bold')  # 设置Y轴标签
# # plt.grid(True)
# # plt.axis('equal')
# plt.savefig('result/experiment2/rrt.png', dpi=600)
# plt.show()
#
#
# ############################################################################################################################################################
# # 确定统一的数组长度
# # desired_length = 100
# #
# # # 确保 trajectory1 和 follow1 的长度相同
# # if len(trajectory1) != len(follow1) or len(trajectory1) != len(sampled_points_array) or len(follow1) != len(sampled_points_array):
# #     # 创建插值函数
# #     f_trajectory1 = interp1d(np.arange(len(trajectory1)), trajectory1, axis=0)
# #     f_follow1 = interp1d(np.arange(len(follow1)), follow1, axis=0)
# #     f_sampled_points_array = interp1d(np.arange(len(sampled_points_array)), sampled_points_array, axis=0)
# #
# #     # 在新长度范围内进行插值
# #     trajectory1_interp = f_trajectory1(np.linspace(0, len(trajectory1) - 1, desired_length))
# #     follow1_interp = f_follow1(np.linspace(0, len(follow1) - 1, desired_length))
# #     sampled_points_array_interp = f_sampled_points_array(np.linspace(0, len(sampled_points_array) - 1, desired_length))
# #
# #     # 更新 trajectory1, follow1 和 sampled_points_array
# #     trajectory1 = trajectory1_interp
# #     follow1 = follow1_interp
# #     sampled_points_array = sampled_points_array_interp
#
#
# # 计算原始轨迹的曲率
# # 计算曲率数组
# curvature_original = []
# for i in range(len(trajectory1) - 2):
#     x = trajectory1[i:i+3, 0]
#     y = trajectory1[i:i+3, 1]
#     kappa= compute_curvature(x, y)
#     curvature_original.append(kappa)
#
# # 将结果转换为数组
# curvature_original = np.array(curvature_original)
# # 计算曲率数组
# curvature_smoothed = []
# for i in range(len(follow1) - 2):
#     x = follow1[i:i+3, 0]
#     y = follow1[i:i+3, 1]
#     kappa= compute_curvature(x, y)
#     curvature_smoothed.append(kappa)
#
# # 将结果转换为数组
# curvature_smoothed = np.array(curvature_smoothed)
#
# curvature_rrt = []
#
# for i in range(len(sampled_points_array) - 2):
#     x = sampled_points_array[i:i+3, 0]
#     y = sampled_points_array[i:i+3, 1]
#     kappa= compute_curvature(x, y)
#     curvature_rrt.append(kappa)
#
# # 将结果转换为数组
# curvature_rrt = np.array(curvature_rrt)
#
# curvature_rrt_bezier = []
# for i in range(len(smoothed_path) - 2):
#     x = smoothed_path[i:i+3, 0]
#     y = smoothed_path[i:i+3, 1]
#     kappa= compute_curvature(x, y)
#     curvature_rrt_bezier.append(kappa)
#
# # 将结果转换为数组
# curvature_rrt_bezier = np.array(curvature_rrt_bezier)
#
# # 找到不是NaN的索引
# valid_indices = ~np.isnan(curvature_smoothed)
#
# # 通过布尔索引删除NaN值
# curvature_smoothed_without_nan = curvature_smoothed[valid_indices]
#
# # 找到删除NaN值后的最大值
# max_value = np.max(curvature_smoothed_without_nan)
#
#
# # 在最大值处画一条垂直于x轴的直线
# # plt.axhline(y=max_value, color='b', linestyle='--')
# # plt.text(20, max_value, f'Max: {max_value:.2f}', ha='right', va='bottom', color='b', fontsize=14)
#
# plt.plot(curvature_rrt, 'k-', label='RRT')
# plt.plot(curvature_rrt_bezier, 'r-', label='RRT+bezier')
# plt.plot(curvature_smoothed, 'b-', label='Ours')
# plt.legend(loc='upper left',fontsize=14)  # Add legend and specify position to upper right
#
# plt.xticks(fontsize=14, weight='bold')
# plt.yticks(fontsize=14, weight='bold')
#
# plt.xlabel('Point Index', fontsize=14, weight='bold')  # 设置X轴标签
# plt.ylabel('Curvature', fontsize=14, weight='bold')  # 设置Y轴标签
# # 保存图片，并自定义分辨率为300dpi
# plt.savefig('result/experiment2/curvature.png', dpi=300)
# plt.show()
#
#
#
# # 将轨迹数据写入CSV文件
# np.savetxt('result/experiment2/curvature_original.csv', curvature_original, delimiter=',', header='X,Y', comments='')
#
# # 将跟随数据写入CSV文件
# np.savetxt('result/experiment2/curvature_smoothed.csv', curvature_smoothed, delimiter=',', header='X,Y', comments='')
#
# # 将平滑路径数据写入CSV文件
# np.savetxt('result/experiment2/curvature_rrt.csv', curvature_rrt, delimiter=',', header='X,Y', comments='')
#
# # 将平滑路径数据写入CSV文件
# np.savetxt('result/experiment2/curvature_rrt_bezier.csv', curvature_rrt_bezier, delimiter=',', header='X,Y', comments='')
#
# frechet_dist = frechet_distance(trajectory1, follow1)
# print(frechet_dist)
# frechet_dist = frechet_distance(trajectory1, sampled_points_array)
# print(frechet_dist)
# frechet_dist = frechet_distance(trajectory1, smoothed_path)
# print(frechet_dist)
#
