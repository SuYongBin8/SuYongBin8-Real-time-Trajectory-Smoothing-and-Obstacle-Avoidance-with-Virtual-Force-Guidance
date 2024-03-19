# 判断线和圆是否相交。计算当前直线点与圆的最近距离
# Determine whether the line intersects with the circle, and calculate the closest distance between the current point on the line and the circle.
import math
import matplotlib.pyplot as plt
import numpy as np

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


#
# #
# # test data
# center = (5, 0)
# radius = 10
# pointA = (1, 10)
# pointB = (0, -5)
#
#
# # Call the function to draw a circle and a line.
# intersection = intersect_line_circle(center, radius, pointA, pointB)
#
# print('Intersection Point 1:')
# print(intersection)
#
# if intersection is not None:
#     plot_circle_and_line(center, radius, pointA, pointB, intersection)