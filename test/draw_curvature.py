import csv
import numpy as np
from matplotlib import pyplot as plt

# 打开 CSV 文件
with open('result/experiment2/curvature_smoothed.csv', 'r') as file:
    # 创建 CSV 读取器
    reader = csv.reader(file)
    # 跳过第一行
    next(reader)
    # 读取每一行，并将第一列的数据存储到列表中
    curvature_smoothed = [float(row[0]) for row in reader]
    curvature_smoothed = curvature_smoothed[1:]
with open('result/experiment2/curvature_rrt.csv', 'r') as file:
    # 创建 CSV 读取器
    reader = csv.reader(file)
    # 跳过第一行
    next(reader)
    # 读取每一行，并将第一列的数据存储到列表中
    curvature_rrt = [float(row[0]) for row in reader]
    curvature_rrt = curvature_rrt[1:]

print(curvature_smoothed)
print(curvature_rrt)
max_value = np.max(curvature_smoothed)
# 在最大值处画一条垂直于x轴的直线
plt.axhline(y=max_value, color='b', linestyle='--')
plt.text(90, max_value-0.21, f'Max: {max_value:.2f}', ha='right', va='bottom', color='b', fontsize=14)


plt.plot(curvature_rrt, 'r-', label='RRT+bezier')
plt.plot(curvature_smoothed, 'b-', label='Ours')
plt.legend(loc='upper left',fontsize=14)  # Add legend and specify position to upper right

plt.xticks(fontsize=14, weight='bold')
plt.yticks(fontsize=14, weight='bold')

plt.xlabel('Point Index', fontsize=14, weight='bold')  # 设置X轴标签
plt.ylabel('Curvature', fontsize=14, weight='bold')  # 设置Y轴标签
# 保存图片，并自定义分辨率为300dpi
plt.savefig('curvature.png', dpi=600)
plt.show()


