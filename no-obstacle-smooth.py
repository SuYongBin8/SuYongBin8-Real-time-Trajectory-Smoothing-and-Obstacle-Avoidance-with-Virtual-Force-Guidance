# Routine for real-time trajectory smoothing in the absence of obstacles.

import numpy as np
import matplotlib.pyplot as plt

from force import calculate_mov_direction

# Define some parameters
kat1 = 1
kat2 = 0.5
kat3 = 0.1
kre1 = 0.01
kre2 = 0.01
kre3 = 0.01
step_max = 0.15
disErr = 1

# 创建一个新的图形
plt.figure()

# 绘制网格
plt.grid(True)

# 设置网格大小为10x6
plt.xticks(range(11))
plt.yticks(range(7))

plt.xlim(-0.5, 10.5)
plt.ylim(-0.5, 6)


# Generate a sawtooth wave trajectory
x = np.linspace(0, 10, 200)  # Generate 200 evenly spaced points between 0 and 10
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

    trajectory1 = np.array(trajectory)

    # Plot trajectory and follow data, and add legend
    plt.plot(trajectory1[:, 0], trajectory1[:, 1], 'yo-', lw=2, markersize=3, label='Original trajectory')
    follow1 = np.array(follow)
    plt.plot(follow1[:, 0], follow1[:, 1], 'bo-', lw=2, markersize=3, label='Smoothed trajectory')

    # Refresh the plot
    plt.draw()
    plt.pause(0.05)

    if count == 0:
        plt.legend(loc='upper right')  # Add legend and specify position to upper right
    count = count + 1

plt.show()
