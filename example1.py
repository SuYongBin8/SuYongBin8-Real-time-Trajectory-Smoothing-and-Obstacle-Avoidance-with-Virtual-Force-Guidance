# Routine for real-time trajectory smoothing in the absence of obstacles.

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from basic import calculate_mov_direction, compute_curvature, frechet_distance

# Define some parameters
kat1 = 1
kat2 = 0.5
kat3 = 0.1
kre1 = 0.01
kre2 = 0.01
kre3 = 0.01
step_max = 0.15#0.1   0.15
disErr = 3 # 2

# 创建一个新的图形
figsize=(10, 8)

# 绘制网格
plt.grid(True)

# 设置网格大小为10x6
plt.xticks(range(6),fontsize=14, weight='bold')
plt.yticks(range(7),fontsize=14, weight='bold')

plt.xlim(-0.5, 5.5)
plt.ylim(-0.5, 6)
plt.xlabel('X', fontsize=14, weight='bold')  # 设置X轴标签
plt.ylabel('Y', fontsize=14, weight='bold')  # 设置Y轴标签


# Generate a sawtooth wave trajectory
x = np.linspace(0, 5, 100)  # Generate 200 evenly spaced points between 0 and 10
y = np.zeros_like(x)  # Initialize y array to all zeros

# Generate the sawtooth wave
period = 2.5  # Period of the sawtooth wave
amplitude = 5  # Amplitude of the sawtooth wave
for i in range(len(x)):
    phase = x[i] % period  # Calculate the phase for each point (0-2.5)
    if phase <= period / 2:
        y[i] = amplitude * phase / (period / 2)  # Increasing ramp
    else:
        y[i] = amplitude - amplitude * (phase - period / 2) / (period / 2)  # Decreasing ramp

# Combine x and y into a 2D array
Traj = np.column_stack((x, y))

# Plot the start and end points
start = Traj[0]
goal = Traj[-1]

plt.plot(start[0], start[1], 'bo', markersize=10)  # Start point
plt.plot(goal[0], goal[1], 'ro', markersize=10)  # End point

# Initialize the position and trajectory of the moving point
position = np.array(start)
trajectory = [position]

# Initialize the position and trajectory of the follower
obj_point = start
follow = [obj_point]

i = 0
count = 0


# Move the point until it reaches close to the goal
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

    # Calculate the virtual force exerted by the original trajectory on the "follower".
    if i < 3:  # The first three points only provide attraction force
        delta = calculate_mov_direction(b3, b2, b1, obj_point, f1, f2, f3, kat1, 0, 0, 0, 0, 0)
    elif i > Traj.shape[0] - 2:  # The last three points only provide attraction force
        delta = calculate_mov_direction(b3, b2, b1, obj_point, f1, f2, f3, kat1, 0, 0, 0, 0, 0)
    else:  # Normal calculation for intermediate points
        delta = calculate_mov_direction(b3, b2, b1, obj_point, f1, f2, f3, kat1, kat2, kat3, kre1, kre2, kre3)

    # Limit the step length
    length = np.linalg.norm(delta)
    if length > step_max:
        delta = step_max / length * delta

    # Calculate the next point
    obj_point = obj_point + delta
    follow.append(obj_point)

    # Update position and record trajectory
    position = np.array(f1)
    trajectory.append(position)

    # Check if within tracking range, if not, wait
    if np.linalg.norm(obj_point - position) < disErr:
        i = i + 1
    else:
        if count % 4 == 0:
            i = i + 1


    trajectory1 = np.array(trajectory)

    # Plot trajectory and follow data, and add legend
    plt.plot(trajectory1[:, 0], trajectory1[:, 1], 'yo-', lw=2, markersize=3, label='Original trajectory')
    follow1 = np.array(follow)
    plt.plot(follow1[:, 0], follow1[:, 1], 'bo-', lw=2, markersize=3, label='Smoothed trajectory')

    # Refresh the plot
    plt.draw()
    plt.pause(0.05)

    if count == 0:
        plt.legend(loc='upper right',fontsize=15)  # Add legend and specify position to upper right



    count = count + 1

# 保存图片，并自定义分辨率为300dpi
plt.savefig('result/experiment1/trajectory.png', dpi=600)
plt.show()

# 确定统一的数组长度
desired_length = 100

# 确保 trajectory1 和 follow1 的长度相同
if len(trajectory1) != len(follow1):
    # 创建插值函数
    f_trajectory1 = interp1d(np.arange(len(trajectory1)), trajectory1, axis=0)
    f_follow1 = interp1d(np.arange(len(follow1)), follow1, axis=0)

    # 在新长度范围内进行插值
    trajectory1_interp = f_trajectory1(np.linspace(0, len(trajectory1) - 1, desired_length))
    follow1_interp = f_follow1(np.linspace(0, len(follow1) - 1, desired_length))

    # 更新 trajectory1 和 follow1
    trajectory1 = trajectory1_interp
    follow1 = follow1_interp

# 计算原始轨迹的曲率
# 计算曲率数组
curvature_original = []
for i in range(len(trajectory1) - 2):
    x = trajectory1[i:i+3, 0]
    y = trajectory1[i:i+3, 1]
    kappa= compute_curvature(x, y)
    curvature_original.append(kappa)

# 将结果转换为数组
curvature_original = np.array(curvature_original)
# 计算曲率数组
curvature_smoothed = []
for i in range(len(follow1) - 2):
    x = follow1[i:i+3, 0]
    y = follow1[i:i+3, 1]
    kappa= compute_curvature(x, y)
    curvature_smoothed.append(kappa)

# 将结果转换为数组
curvature_smoothed = np.array(curvature_smoothed)

# curvature_original = compute_curvature(trajectory1)
# curvature_original = curvature_original[2:]
# # 计算平滑后轨迹的曲率
# curvature_smoothed = compute_curvature(follow1)
# curvature_smoothed = curvature_smoothed[2:]
# 找到不是NaN的索引
valid_indices = ~np.isnan(curvature_smoothed)

# 通过布尔索引删除NaN值
curvature_smoothed_without_nan = curvature_smoothed[valid_indices]

# 找到删除NaN值后的最大值
max_value = np.max(curvature_smoothed_without_nan)


# 在最大值处画一条垂直于x轴的直线
plt.axhline(y=max_value, color='b', linestyle='--')
plt.text(107, max_value, f'Max: {max_value:.2f}', ha='right', va='bottom', color='b', fontsize=14)

plt.plot(curvature_original, 'r-', label='Original trajectory curvature')
plt.plot(curvature_smoothed, 'b-', label='Smoothed trajectory curvature')
plt.legend(loc='upper right',fontsize=15)  # Add legend and specify position to upper right

plt.xticks(fontsize=14, weight='bold')
plt.yticks(fontsize=14, weight='bold')

plt.xlabel('Point Index', fontsize=14, weight='bold')  # 设置X轴标签
plt.ylabel('Curvature', fontsize=14, weight='bold')  # 设置Y轴标签
# 保存图片，并自定义分辨率为300dpi
plt.savefig('result/experiment1/curvature-15.png', dpi=600)
plt.show()

# 将跟随数据写入CSV文件
np.savetxt('result/experiment1/curvature_original.csv', curvature_original, delimiter=',', header='X,Y', comments='')
np.savetxt('result/experiment1/curvature_smoothed-15.csv', curvature_smoothed, delimiter=',', header='X,Y', comments='')
frechet_dist = frechet_distance(trajectory1, follow1)
print(frechet_dist)